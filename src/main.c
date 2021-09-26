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
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"
#include "u8g2.h"

#include "main.h"
#include "utils.h"
#include "u8g2_esp32_hal.h"

#define CMD_BUFFER_SIZE (3072)
#define TOD_BUFFER_SIZE (1024)
#define UART_PORT_NUM (UART_NUM_2)

static void uart_receive_task(void *pvParameters);
static void parse_tod_task(void *pvParameters);
static void parse_cmd_task(void *pvParameters);
static void update_display_task(void *pvParameters);
static void send_cmd_task(void *pvParameters);
static void uart_init();
static void display_init();

// The object for the GLCD display
u8g2_t u8g2;

// Message queues
static QueueHandle_t queue_uart2;
static QueueHandle_t queue_tod;
static QueueHandle_t queue_cmd;
static QueueHandle_t can_send_cmd;

// UART message ring buffer
RingbufHandle_t buf_handle;

// Object that holds the state of the GPSDO
gpsdo_state_t gpsdo_state = {
    .manufacturer = "",
    .serial_number = "",
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
    .alarm_hw = "",
    .alarm_op = "",
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
    static const char *TAG = "main";

    display_init();
    uart_init();

    initialize_uccm();

    vTaskDelay(5000 / portTICK_PERIOD_MS);

    can_send_cmd = xSemaphoreCreateBinary();

    queue_tod = xQueueCreate(10, sizeof(char *));
    if (queue_tod == NULL)
    {
        ESP_LOGI(TAG, "Failed to create queue_tod");
    }
    xTaskCreate(parse_tod_task, "parse_tod_task", 10240, NULL, 6, NULL);

    queue_cmd = xQueueCreate(10, sizeof(char *));
    if (queue_cmd == NULL)
    {
        ESP_LOGI(TAG, "Failed to create queue_cmd");
    }
    // xTaskCreate(parse_cmd_task, "parse_cmd_task", 10240, NULL, 3, NULL);

    // xTaskCreate(update_display_task, "updateDisplayTask", 10240, NULL, 1, NULL);

    xTaskCreate(uart_receive_task, "uart_receive_task", 20480, NULL, 12, NULL);

    // xTaskCreate(send_cmd_task, "send_cmd_task", 20480, NULL, 2, NULL);
}

void initialize_uccm()
{
    uart_write_bytes(UART_PORT_NUM, "\n", sizeof("\n") - 1);
    uart_write_bytes(UART_PORT_NUM, "TOD EN\n", sizeof("TOD EN\n") - 1);
}

static void send_cmd_task(void *pvParameters)
{
    static const char *TAG = "send_cmd_task";
    char *commands[13] = {
        "*IDN?",
        "ALAR:HARD?",
        "ALAR:OPER?",
        "DIAG:LOOP?",
        "DIAG:ROSC:EFC:REL?",
        "DIAG:ROSC:EFC:DATA?",
        "GPS:POS?",
        "LED:GPSL?",
        "OUTP:STAT?",
        "PULLINRANGE?",
        "SYNC:FFOM?",
        "SYNC:TINT?"
        // "SYST:STAT?"
    };

    for (;;)
    {
        for (int i = 0; i < 13; i++)
        {
            xSemaphoreTake(can_send_cmd, portMAX_DELAY);
            ESP_LOGI(TAG, "Sending command %s", commands[i]);
            uart_write_bytes(UART_PORT_NUM, commands[i], strlen(commands[i]));
            uart_write_bytes(UART_PORT_NUM, "\n", sizeof("\n") - 1);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
    vTaskDelete(NULL);
}

static void parse_cmd_task(void *pvParameters)
{
    static const char *TAG = "parse_cmd_task";
    char *cmd_data;
    char *lf_pos = NULL;
    char *complete_pos = NULL;
    for (;;)
    {
        if (xQueueReceive(queue_cmd, &cmd_data, (portTickType)portMAX_DELAY))
        {
            ESP_LOGI(TAG, "cmd data: %s", cmd_data);
            lf_pos = strstr(cmd_data, "\r\n");
            complete_pos = strstr(cmd_data, "\"Command Complete\"");
            if (lf_pos != NULL)
            {
                size_t len_cmd = lf_pos - cmd_data;
                size_t len_data = complete_pos - lf_pos;
                char *command = malloc(len_cmd + 1);
                char *data = malloc(len_data + 1);
                bzero(command, len_cmd + 1);
                bzero(data, len_data + 1);
                memcpy(command, cmd_data, len_cmd);
                memcpy(data, lf_pos, len_data);
                // Parse the received command result
                parse_command(&gpsdo_state, command, data);
                free(command);
                free(data);
            }
            free(cmd_data);
            cmd_data = NULL;
        }
    }
    vTaskDelete(NULL);
}

static void parse_tod_task(void *pvParameters)
{
    static const char *TAG = "parse_tod_task";
    char *tod_data;
    char *s;
    int vals[50];
    int i, j = 0;
    uint32_t gpsepoch, utctime;
    for (;;)
    {
        if (xQueueReceive(queue_tod, &tod_data, (portTickType)portMAX_DELAY))
        {
            // ESP_LOGI(TAG, "TOD data: %s", tod_data);
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
    static const char *TAG = "uart_receive_task";
    uart_event_t event;
    char *dtmp = malloc(TOD_BUFFER_SIZE);
    char *cmd_buffer = malloc(CMD_BUFFER_SIZE);
    char *tod_buffer = malloc(TOD_BUFFER_SIZE);
    char *prompt_pos;
    char *c5_pos;
    char *ca_pos;
    int cmd_buffer_index = 0;
    int tod_buffer_index = 0;
    bool c5_detected = false;

    bzero(tod_buffer, TOD_BUFFER_SIZE);
    bzero(cmd_buffer, CMD_BUFFER_SIZE);

    uart_flush_input(UART_PORT_NUM);

    for (;;)
    {
        //Waiting for UART event.
        if (xQueueReceive(queue_uart2, (void *)&event, (portTickType)portMAX_DELAY))
        {
            bzero(dtmp, TOD_BUFFER_SIZE);
            switch (event.type)
            {
            /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
            case UART_DATA:
                uart_read_bytes(UART_PORT_NUM, dtmp, event.size, portMAX_DELAY);

                // If the c5 has been detected in the previous call, we just get
                // the rest of the data and notify the parsing function.
                if (c5_detected)
                {
                    // We add 2 to obtain the ca string itself
                    ca_pos = strstr((char *)dtmp, "ca");
                    // This is the case where ca_pos is outside of the string. In this case, we bail out.
                    if (ca_pos >= (dtmp + event.size))
                    {
                        ESP_LOGW(TAG, "ca_pos out of boundaries");
                        break;
                    }
                    // If ca wasn't received yet, we need to break/leave the function and wait for the next one
                    if (ca_pos == NULL)
                    {
                        ESP_LOGD(TAG, "ca_pos not found yet. Accumulating.");
                        break;
                    }
                    ca_pos += 2;
                    memcpy(&tod_buffer[tod_buffer_index], dtmp, ca_pos - (char *)dtmp);
                    tod_buffer_index += ca_pos - (char *)dtmp;
                    ESP_LOGD(TAG, "Full TOD size: %d", tod_buffer_index);
                    // Sometimes we can get other outputs in the middle of the TOD.
                    // If that's the case, discard the data and don't notify the parsing function.
                    if (tod_buffer_index == 131)
                    {
                        // Here notify the parsing task that data is ready
                        char *tod_data = malloc(strlen(tod_buffer) + 1);
                        strcpy(tod_data, tod_buffer);
                        if (xQueueSendToBack(queue_tod, &tod_data, (portTickType)portMAX_DELAY) != pdPASS)
                        {
                            ESP_LOGE(TAG, "Error sending data to TOD queue");
                        }
                        xSemaphoreGive(can_send_cmd);
                    }
                    tod_buffer_index = 0;
                    // Zero out the buffer to prevent nastiness
                    // bzero(tod_buffer, TOD_BUFFER_SIZE);
                    c5_detected = false;
                    ESP_LOGD(TAG, "Cleaned up C5 detected");

                    break;
                }

                // Check if the c5 string is in the buffer. If yes, wait another round and
                // then ship the data to the parsing function.'
                c5_pos = strstr((char *)dtmp, "c5");
                if (c5_pos != NULL)
                {
                    bzero(tod_buffer, TOD_BUFFER_SIZE);
                    c5_detected = true;
                    ESP_LOGD(TAG, "C5 detected");
                    int data_length = event.size - (c5_pos - dtmp);
                    memcpy(&tod_buffer[0], c5_pos, data_length);
                    tod_buffer_index = data_length;
                    break;
                }

                // In the case of other commands, we just copy data into the buffer
                // until we get the prompt again.
                // ESP_LOGI(TAG, "uart data: %s", dtmp);
                // memcpy(&cmd_buffer[cmd_buffer_index], dtmp, event.size);
                // cmd_buffer_index += event.size;

                // prompt_pos = strstr(cmd_buffer, "UCCM> ");
                // if (prompt_pos != NULL)
                // {
                //     char *cmd_start = cmd_buffer;
                //     // This loop is a trick to split the commands when more than one is received at once and
                //     // send them individually to be processed by the task.
                //     while (prompt_pos != NULL)
                //     {
                //         size_t len = prompt_pos - cmd_start;
                //         char *cmd_data = malloc(len + 1);
                //         bzero(cmd_data, len + 1);
                //         memcpy(cmd_data, cmd_start, len);
                //         if (xQueueSendToBack(queue_cmd, &cmd_data, (portTickType)portMAX_DELAY) != pdPASS)
                //         {
                //             ESP_LOGE(TAG, "Error sending data to cmd queue");
                //         }
                //         xSemaphoreGive(can_send_cmd);
                //         cmd_start += len + 6;
                //         prompt_pos = strstr(cmd_start, "UCCM> ");
                //     }
                //     // Sometimes we get partial commands that don't have the UCCM> part yet, so we get a remaining part.
                //     // This copies the remaining to the start of the array, sets the array index to the length of the remain
                //     // and zero out the rest.
                //     // That way, when we receive the rest, we can still process it as a command.
                //     memcpy(cmd_buffer, cmd_start, strlen(cmd_start));
                //     cmd_buffer_index = strlen(cmd_start);
                //     bzero(&(cmd_buffer[cmd_buffer_index]), CMD_BUFFER_SIZE - cmd_buffer_index);
                //     break;
                // }

                break;
            //Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGW(TAG, "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(UART_PORT_NUM);
                xQueueReset(queue_uart2);
                break;
            //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGW(TAG, "ring buffer full");
                // If buffer full happened, you should consider increasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(UART_PORT_NUM);
                xQueueReset(queue_uart2);
                break;
            //Event of UART RX break detected
            case UART_BREAK:
                ESP_LOGI(TAG, "uart rx break");
                break;
            //Event of UART parity check error
            case UART_PARITY_ERR:
                ESP_LOGW(TAG, "uart parity error");
                break;
            //Event of UART frame error
            case UART_FRAME_ERR:
                ESP_LOGW(TAG, "uart frame error");
                break;
                //UART_PATTERN_DET
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
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));

    //Set UART pins (using UART0 default pins ie no changes.)
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, GPIO_NUM_17, GPIO_NUM_16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    //Install UART driver, and get the queue.
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, CMD_BUFFER_SIZE * 2, 0, 20, &queue_uart2, 0));

    /* Set pattern interrupt, used to detect the end of a line */
    // ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(UART_PORT_NUM, '\n', 1, 9, 0, 0));
    /* Set pattern queue size */
    // ESP_ERROR_CHECK(uart_pattern_queue_reset(UART_PORT_NUM, 20));
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
    char *holder = malloc(24 * sizeof(char));
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_6x12_tf);
    // Drawing of left side
    sprintf(holder, "UCCM:%.10s %.7s", gpsdo_state.manufacturer, gpsdo_state.model);
    u8g2_DrawStr(&u8g2, 0, 7, holder);
    sprintf(holder, "SN:%.10s,%.9s", gpsdo_state.serial_number, gpsdo_state.version);
    u8g2_DrawStr(&u8g2, 0, 15, holder);
    sprintf(holder, "Temp:%6.3f", gpsdo_state.temperature);
    u8g2_DrawStr(&u8g2, 0, 23, holder);
    sprintf(holder, "DAC:%+7.4f %%", gpsdo_state.dac);
    u8g2_DrawStr(&u8g2, 0, 31, holder);
    sprintf(holder, "Phase:%+5.2fns", gpsdo_state.phase);
    u8g2_DrawStr(&u8g2, 0, 39, holder);
    sprintf(holder, "PPS:%+7.4fns", gpsdo_state.pps);
    u8g2_DrawStr(&u8g2, 0, 47, holder);
    sprintf(holder, "FREQ DIFF:%5.2f", gpsdo_state.freq_diff);
    u8g2_DrawStr(&u8g2, 0, 55, holder);
    sprintf(holder, "TFOM:%d FFOM:%d", gpsdo_state.tfom, gpsdo_state.ffom);
    u8g2_DrawStr(&u8g2, 0, 63, holder);
    u8g2_SendBuffer(&u8g2);
    free(holder);
    holder = NULL;
}

void monitorScreen()
{
    char *holder = malloc(24 * sizeof(char));
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
    sprintf(holder, "|HW:%.4s", gpsdo_state.alarm_hw);
    u8g2_DrawStr(&u8g2, 81, 55, holder);
    sprintf(holder, "|OP:%.4s", gpsdo_state.alarm_op);
    u8g2_DrawStr(&u8g2, 81, 63, holder);
    u8g2_SendBuffer(&u8g2);
    free(holder);
    holder = NULL;
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
