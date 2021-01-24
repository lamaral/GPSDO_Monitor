#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <esp_system.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "u8g2.h"

#include "main.h"
#include "utils.h"
#include "u8g2_esp32_hal.h"

#define CMD_BUFFER_SIZE (3072)
#define TOD_BUFFER_SIZE (512)
#define ECHO_UART_PORT_NUM (UART_NUM_2)

// Used for debug output
static const char *TAG = "xableta";

// The object for the GLCD display
u8g2_t u8g2;

// Message queues
static QueueHandle_t queue_uart2;
static QueueHandle_t queue_tod;
static QueueHandle_t queue_cmd;

// UART message ring buffer
RingbufHandle_t buf_handle;

// Object that holds the state of the GPSDO
gpsdo_state_t gpsdo_state = {
    .temperature = 0.0,
    .dac = 0.0,
    .phase = 0.0,
    .pps = 0.0,
    .freq_diff = 0.0,
    .tfom = 0,
    .ffom = 0,
    .status_output = 0,
    .status_gps = 0,
    .status_pos = 0,
    .status_opr = 0,
    .alarm_hw = 0,
    .alarm_op = 0,
    .week = 0,
    .tow = 0,
    .utc_offset = 0,
    .altitude = 0.0,
    .latitude = 0.0,
    .longitude = 0.0,
};

// Screen functions pointer array
void (*screen_functions[4])(void) = {&monitorScreen, &uccmDataScreen, &satellitesScreen, &statScreen};

void app_main()
{
    // display_init();
    uart_init();

    // initialize_uccm();

    queue_tod = xQueueCreate(10, sizeof(char *));
    if (queue_tod == NULL)
    {
        ESP_LOGI(TAG, "Failed to create queue_tod");
    }
    xTaskCreate(tod_parse_task, "tod_parse_task", 10240, NULL, 2, NULL);

    queue_cmd = xQueueCreate(10, sizeof(char *));
    if (queue_cmd == NULL)
    {
        ESP_LOGI(TAG, "Failed to create queue_cmd");
    }
    xTaskCreate(cmd_parse_task, "cmd_parse_task", 10240, NULL, 2, NULL);

    // xTaskCreate(update_display_task, "updateDisplayTask", 10240, NULL, 1, NULL);

    xTaskCreate(uart_receive_task, "uart_receive_task", 20480, NULL, 12, NULL);
}

static void cmd_parse_task(void *pvParameters)
{
    char *cmd_data;
    for (;;)
    {
        if (xQueueReceive(queue_cmd, &cmd_data, (portTickType)portMAX_DELAY))
        {
            ESP_LOGI(TAG, "cmd data: %s", cmd_data);
            free(cmd_data);
            cmd_data = NULL;
        }
    }
    vTaskDelete(NULL);
}

static void tod_parse_task(void *pvParameters)
{
    char *tod_data;
    char *s;
    int vals[50];
    int i, j = 0;
    uint32_t gpsepoch, utctime;
    for (;;)
    {
        if (xQueueReceive(queue_tod, &tod_data, (portTickType)portMAX_DELAY))
        {
            ESP_LOGD(TAG, "TOD data: %s", tod_data);
            s = strstr(tod_data, "c5 ");
            if (s == 0)
                goto go_back;
            for (i = 0; i < 131; i += 3)
            { // get the values from the time line
                j = i / 3;
                vals[j] = atohex(&s[i]);
            }
            // Process the timestamp from the message
            // Fields 27 to 30 contain a 32bit timestamp
            gpsepoch = (uint32_t)((vals[27] * (256 * 256 * 256)) + (vals[28] * (256 * 256)) + (vals[29] * (256)) + vals[30]);

            // We need the utc_offset to calculate the UTC time from GPS time
            gpsdo_state.utc_offset = vals[32];

            // Difference between GPS and UTC epoch
            // UTC Offset contains the leap seconds
            utctime = gpsepoch - gpsdo_state.utc_offset;
            utctime += 315964800;
            ESP_LOGD(TAG, "UTC Time: %d", utctime);

            gpsdo_state.week = (int)(gpsepoch / (7 * 24 * 60 * 60));
            ESP_LOGD(TAG, "GPS Week: %d", gpsdo_state.week);

            // vals[33]: 40=PPS validity?  41:phase settling  50:pps invalid?
            //           60:stable  62:stable, leap pending?
            // on power up: 41 -> 43 -> 63 -> 60/62 (62=leap pending?) Trimble
            // on power up: 41 -> 43 -> 60 -> 62 (62=leap pending?)    Trimble UCCM-P

            // vals[34]: 04=normal 0C=antenna open/shorted  06=normal?
            // on power up: 00 -> 04
            // disconnect antenna: 04 -> 0C
            // reconnect antenna   0C -> 04

            // vals[35]: 41=power up  4F=FFOM >0/settling/no antenna    45:FFOM 0,locked? - Trimble
            // on power up: 4F -> 45           Trimble UCCM-P
            // on power up: 41 -> 4F -> 45     Trimble UCCM
            // on antenna disconnect 45 -> 4F  Trimble
            // on antenna connect    4F -> 45

            // vals[36]: 80=have date?/normal  90:date invalid?/no antenna
            // power up 90 -> 80                   Trimble
            // disconnect antenna: 80 -> 90        Trimble UCCM-P
            // reconnect antenna:  90 -> 80        Trimble UCCM-P
        go_back:
            free(tod_data);
            tod_data = NULL;
        }
    }
    vTaskDelete(NULL);
}

void initialize_uccm()
{
    uart_write_bytes(ECHO_UART_PORT_NUM, "TOD EN\r\n", sizeof("TOD EN\r\n"));
}

static void update_display_task(void *pvParameters)
{
    for (;;)
    {
        for (int i = 0; i < 4; i++)
        {
            screen_functions[i]();

            vTaskDelay(5000 / portTICK_PERIOD_MS);
        }
    }
}

static void uart_receive_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t *dtmp = (uint8_t *)malloc(TOD_BUFFER_SIZE);
    char *cmd_buffer = malloc(CMD_BUFFER_SIZE);
    char *tod_buffer = malloc(TOD_BUFFER_SIZE);
    char *prompt_pos;
    int cmd_buffer_index = 0, tod_buffer_index = 0;
    bool c5_detected = false;

    bzero(tod_buffer, TOD_BUFFER_SIZE);
    bzero(cmd_buffer, CMD_BUFFER_SIZE);

    for (;;)
    {
        //Waiting for UART event.
        if (xQueueReceive(queue_uart2, (void *)&event, (portTickType)portMAX_DELAY))
        {
            bzero(dtmp, TOD_BUFFER_SIZE);
            ESP_LOGD(TAG, "uart[%d] event: %d", ECHO_UART_PORT_NUM, event.size);
            switch (event.type)
            {
            /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
            case UART_DATA:
                uart_read_bytes(ECHO_UART_PORT_NUM, dtmp, event.size, portMAX_DELAY);

                // Check if the c5 string is in the buffer. If yes, wait another round and
                // then ship the data to the parsing function.'
                if (strstr((char *)dtmp, "c5") != NULL)
                {
                    c5_detected = true;
                    memcpy(&tod_buffer[tod_buffer_index], dtmp, event.size);
                    tod_buffer_index += event.size;
                    ESP_LOGD(TAG, "C5 detected");
                    break;
                }

                // If the c5 has been detected in the previous call, we just get
                // the rest of the data and notify the parsing function.
                if (c5_detected)
                {
                    memcpy(&tod_buffer[tod_buffer_index], dtmp, event.size);
                    tod_buffer_index += event.size;
                    ESP_LOGD(TAG, "Rest of TOD [%d]: %s", tod_buffer_index, tod_buffer);
                    // Sometimes we can get other outputs in the middle of the TOD.
                    // If that's the case, discard the data and don't notify the parsing function.
                    if (tod_buffer_index == 135)
                    {
                        // Here notify the parsing task that data is ready
                        char *tod_data = malloc(strlen(tod_buffer) + 1);
                        strcpy(tod_data, tod_buffer);
                        if (xQueueSendToBack(queue_tod, &tod_data, (portTickType)portMAX_DELAY) != pdPASS)
                        {
                            ESP_LOGE(TAG, "Error sending data to TOD queue");
                        }
                    }
                    tod_buffer_index = 0;
                    // Zero out the buffer to prevent nastiness
                    bzero(tod_buffer, TOD_BUFFER_SIZE);
                    c5_detected = false;

                    break;
                }

                // In the case of other commands, we just copy data into the buffer
                // until we get the prompt again.
                prompt_pos = strstr((char *)dtmp, "UCCM>");
                if (prompt_pos != NULL)
                {
                    memcpy(&cmd_buffer[cmd_buffer_index], dtmp, event.size);
                    cmd_buffer_index += event.size;

                    char *cmd_data = malloc(cmd_buffer_index + 1);
                    bzero(cmd_data, cmd_buffer_index + 1);
                    memcpy(cmd_data, cmd_buffer, cmd_buffer_index);
                    if (xQueueSendToBack(queue_cmd, &cmd_data, (portTickType)portMAX_DELAY) != pdPASS)
                    {
                        ESP_LOGE(TAG, "Error sending data to cmd queue");
                    }
                    cmd_buffer_index = 0;
                    // Zero out the buffer to prevent nastiness
                    bzero(cmd_buffer, CMD_BUFFER_SIZE);
                    break;
                }
                else
                {
                    
                    break;
                }

                break;
            //Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGI(TAG, "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(ECHO_UART_PORT_NUM);
                xQueueReset(queue_uart2);
                break;
            //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGI(TAG, "ring buffer full");
                // If buffer full happened, you should consider increasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(ECHO_UART_PORT_NUM);
                xQueueReset(queue_uart2);
                break;
            //Event of UART RX break detected
            case UART_BREAK:
                ESP_LOGI(TAG, "uart rx break");
                break;
            //Event of UART parity check error
            case UART_PARITY_ERR:
                ESP_LOGI(TAG, "uart parity error");
                break;
            //Event of UART frame error
            case UART_FRAME_ERR:
                ESP_LOGI(TAG, "uart frame error");
                break;
                //UART_PATTERN_DET
            case UART_PATTERN_DET:
                uart_get_buffered_data_len(ECHO_UART_PORT_NUM, &buffered_size);
                int pos = uart_pattern_pop_pos(ECHO_UART_PORT_NUM);
                ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                if (pos == -1)
                {
                    // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                    // record the position. We should set a larger queue size.
                    // As an example, we directly flush the rx buffer here.
                    uart_flush_input(ECHO_UART_PORT_NUM);
                }
                else
                {
                    uart_read_bytes(ECHO_UART_PORT_NUM, dtmp, pos, 100 / portTICK_PERIOD_MS);
                    uint8_t pat[2];
                    memset(pat, 0, sizeof(pat));
                    uart_read_bytes(ECHO_UART_PORT_NUM, pat, 1, 100 / portTICK_PERIOD_MS);
                    // ESP_LOGI(TAG, "read data: %s", dtmp);
                }
                break;
            //Others
            default:
                break;
            }
        }
    }
    free(dtmp);
    free(cmd_buffer);
    free(tod_buffer);
    dtmp = NULL;
    cmd_buffer = NULL;
    tod_buffer = NULL;
    vTaskDelete(NULL);
}

static void uart_init()
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 57600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    //Install UART driver, and get the queue.
    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, TOD_BUFFER_SIZE * 2, TOD_BUFFER_SIZE * 2, 20, &queue_uart2, 0));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));

    //Set UART pins (using UART0 default pins ie no changes.)
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, GPIO_NUM_17, GPIO_NUM_16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    /* Set pattern interrupt, used to detect the end of a line */
    // ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(ECHO_UART_PORT_NUM, '\n', 1, 9, 0, 0));
    /* Set pattern queue size */
    // ESP_ERROR_CHECK(uart_pattern_queue_reset(ECHO_UART_PORT_NUM, 20));
}

static void display_init()
{
    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.clk = GPIO_NUM_14;
    u8g2_esp32_hal.mosi = GPIO_NUM_13;
    u8g2_esp32_hal.cs = GPIO_NUM_15;
    u8g2_esp32_hal.reset = GPIO_NUM_27;
    u8g2_esp32_hal_init(u8g2_esp32_hal);

    u8g2_Setup_st7920_s_128x64_f(&u8g2, U8G2_R2, u8g2_esp32_spi_byte_cb, u8g2_esp32_gpio_and_delay_cb);

    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0); // wake up display
}

void uccmDataScreen()
{
    // u8g2_FirstPage(&u8g2);
    // do
    // {
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_6x12_tf);
    // Drawing of left side
    u8g2_DrawStr(&u8g2, 0, 7, "UCCM: Samsung");
    u8g2_DrawStr(&u8g2, 0, 15, "SN:S3ZB220090,1.0.0.5");
    u8g2_DrawStr(&u8g2, 0, 23, "Temp: 43.162C");
    u8g2_DrawStr(&u8g2, 0, 31, "DAC: -8.0940 %");
    u8g2_DrawStr(&u8g2, 0, 39, "Phase: +15.70ns");
    u8g2_DrawStr(&u8g2, 0, 47, "PPS: +10.3000ns");
    u8g2_DrawStr(&u8g2, 0, 55, "FREQ DIFF:N/A");
    u8g2_DrawStr(&u8g2, 0, 63, "TFOM: 2 FFOM: Stable");
    u8g2_SendBuffer(&u8g2);
    // } while (u8g2_NextPage(&u8g2));
}

void monitorScreen()
{
    // u8g2_FirstPage(&u8g2);
    // do
    // {
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_6x12_tf);
    // Drawing of left side
    u8g2_DrawStr(&u8g2, 0, 7, "DK2IP GPSDO Monitor");
    u8g2_DrawStr(&u8g2, 0, 15, "14:53:36U");
    u8g2_DrawStr(&u8g2, 0, 23, "Freq: N/A");
    u8g2_DrawStr(&u8g2, 0, 31, "GPSDO Status");
    u8g2_DrawStr(&u8g2, 0, 39, "OUT: Norm");
    u8g2_DrawStr(&u8g2, 0, 47, "GPS: Lock");
    u8g2_DrawStr(&u8g2, 0, 55, "Pos: Hold");
    u8g2_DrawStr(&u8g2, 0, 63, "OPR: Active");
    // Drawing of right side
    u8g2_DrawStr(&u8g2, 63, 15, "25 Apr 2020");
    u8g2_DrawStr(&u8g2, 81, 39, "|Alarm");
    u8g2_DrawStr(&u8g2, 81, 47, "|------");
    u8g2_DrawStr(&u8g2, 81, 55, "|HW:NONE");
    u8g2_DrawStr(&u8g2, 81, 63, "|OP:NONE");
    u8g2_SendBuffer(&u8g2);
    // } while (u8g2_NextPage(&u8g2));
}

void satellitesScreen()
{
    // u8g2_FirstPage(&u8g2);
    // do
    // {
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_6x12_tf);
    // Drawing of left side
    u8g2_DrawStr(&u8g2, 0, 7, "Tracking: 30");
    u8g2_DrawStr(&u8g2, 0, 15, "Visible: 10");
    u8g2_DrawStr(&u8g2, 0, 23, " PRN E1  AZ  C/N Sig.");
    u8g2_DrawStr(&u8g2, 0, 31, "  2  57  33   42  42");
    u8g2_DrawStr(&u8g2, 0, 39, "  5  57 336   50  50");
    u8g2_DrawStr(&u8g2, 0, 47, " 15  23 206   36  36");
    u8g2_DrawStr(&u8g2, 0, 55, " 19  22 149  N/A  --");
    u8g2_DrawStr(&u8g2, 0, 63, " 25  13 274  N/A  --");
    u8g2_SendBuffer(&u8g2);
    // } while (u8g2_NextPage(&u8g2));
}

void statScreen()
{
    // u8g2_FirstPage(&u8g2);
    // do
    // {
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_6x12_tf);
    // Drawing of left side
    u8g2_DrawStr(&u8g2, 0, 7, "14:53:36 UTC");
    u8g2_DrawStr(&u8g2, 0, 15, "25 Apr 2020");
    u8g2_DrawStr(&u8g2, 0, 23, "Week:  2050");
    u8g2_DrawStr(&u8g2, 0, 31, "Tow: 399234");
    u8g2_DrawStr(&u8g2, 0, 39, "UTC ofs: 18");
    u8g2_DrawStr(&u8g2, 0, 47, "Alt:+134.300 m");
    u8g2_DrawStr(&u8g2, 0, 55, "Lat:N 22.5446933");
    u8g2_DrawStr(&u8g2, 0, 63, "Lon:E 114.0800450");
    // Drawing of right side
    u8g2_DrawStr(&u8g2, 81, 7, "GPS STAT");
    u8g2_DrawStr(&u8g2, 81, 15, "OUT:Norm");
    u8g2_DrawStr(&u8g2, 81, 23, "GPS:Lock");
    u8g2_DrawStr(&u8g2, 81, 31, "Pos:Hold");
    u8g2_DrawStr(&u8g2, 81, 39, "Stable");
    u8g2_SendBuffer(&u8g2);
    // } while (u8g2_NextPage(&u8g2));
}
