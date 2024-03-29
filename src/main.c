#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <time.h>
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
#define TOD_BUFFER_SIZE (256)
#define TOD_PACKET_SIZE (44)
#define TOD_PORT_NUM (UART_NUM_1)
#define CMD_PORT_NUM (UART_NUM_2)

static void uart_receive_tod_task(void *pvParameters);
static void uart_receive_cmd_task(void *pvParameters);
static void parse_tod_task(void *pvParameters);
static void parse_cmd_task(void *pvParameters);
static void update_display_task(void *pvParameters);
static void send_cmd_task(void *pvParameters);
static void initialize_uart();
static void initialize_display();

// The object for the GLCD display
u8g2_t u8g2;

// Message queues
static QueueHandle_t queue_uart_cmd;
static QueueHandle_t queue_uart_tod;
static QueueHandle_t queue_cmd;
static QueueHandle_t queue_tod;
static SemaphoreHandle_t can_send_cmd;

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
    .status_output = "",
    .status_gps = "",
    .status_pos = "",
    .status_opr = "",
    .alarm_hw = "",
    .alarm_op = "",
    .week = 0,
    .tow = 0,
    .utc_offset = 0,
    .date = "01 Jan 1970",
    .time = "00:00:00 U",
    .altitude = 0.0,
    .latitude = 0.0,
    .longitude = 0.0,
};

// Screen functions pointer array
void (*screen_functions[4])(void) = {&monitorScreen, &uccmDataScreen, &satellitesScreen, &statScreen};

void app_main()
{
    static const char *TAG = "main";

    initialize_display();
    initialize_uart();

    initialize_uccm();

    ESP_LOGI(TAG, "Initialization complete. Waiting before creating tasks.");

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    uart_flush_input(CMD_PORT_NUM);

    queue_tod = xQueueCreate(20, sizeof(char *));
    if (queue_tod == NULL)
    {
        ESP_LOGE(TAG, "Failed to create queue_tod");
    }

    queue_cmd = xQueueCreate(20, sizeof(char *));
    if (queue_cmd == NULL)
    {
        ESP_LOGE(TAG, "Failed to create queue_cmd");
    }

    can_send_cmd = xSemaphoreCreateBinary();
    if (can_send_cmd == NULL)
    {
        ESP_LOGE(TAG, "Failed to create can_send_cmd");
    }

    xTaskCreate(parse_tod_task, "parse_tod_task", 2048, NULL, 6, NULL);
    xTaskCreate(parse_cmd_task, "parse_cmd_task", 2048, NULL, 3, NULL);

    xTaskCreate(update_display_task, "updateDisplayTask", 2048, NULL, 1, NULL);

    xTaskCreate(uart_receive_tod_task, "uart_receive_tod_task", 2048, NULL, 12, NULL);
    xTaskCreate(uart_receive_cmd_task, "uart_receive_cmd_task", 2048, NULL, 11, NULL);

    xTaskCreate(send_cmd_task, "send_cmd_task", 2048, NULL, 2, NULL);
}

void initialize_uccm()
{
    uart_write_bytes(CMD_PORT_NUM, "\n", sizeof("\n") - 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    uart_write_bytes(CMD_PORT_NUM, "\n", sizeof("\n") - 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    uart_write_bytes(CMD_PORT_NUM, "SYNC:REF:DISABLE LINK\n", sizeof("SYNC:REF:DISABLE LINK\n") - 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    uart_write_bytes(CMD_PORT_NUM, "SYNC:REF:DISABLE EXT\n", sizeof("SYNC:REF:DISABLE EXT\n") - 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    uart_write_bytes(CMD_PORT_NUM, "SYNC:REF:ENABLE GPS\n", sizeof("SYNC:REF:ENABLE GPS\n") - 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    uart_write_bytes(CMD_PORT_NUM, "REF:TYPE GPS\n", sizeof("REF:TYPE GPS\n") - 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    uart_write_bytes(CMD_PORT_NUM, "OUTP:TP:SEL PP1S\n", sizeof("OUTP:TP:SEL PP1S\n") - 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // uart_write_bytes(CMD_PORT_NUM, "GPS:SAT:TRAC:EMAN 20\n", sizeof("GPS:SAT:TRAC:EMAN 20\n") - 1);
    // vTaskDelay(100 / portTICK_PERIOD_MS);
    uart_write_bytes(CMD_PORT_NUM, "\n", sizeof("\n") - 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    uart_flush_input(CMD_PORT_NUM);
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
        "SYNC:TINT?",
        "SYST:STAT?"};

    for (;;)
    {
        for (int i = 0; i < 13; i++)
        {
            // xSemaphoreTake(can_send_cmd, portMAX_DELAY);
            ESP_LOGD(TAG, "Sending command %s", commands[i]);
            uart_write_bytes(CMD_PORT_NUM, commands[i], strlen(commands[i]));
            uart_write_bytes(CMD_PORT_NUM, "\n", sizeof("\n") - 1);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
    vTaskDelete(NULL);
}

static void parse_cmd_task(void *pvParameters)
{
    static const char *TAG = "parse_cmd_task";
    esp_log_level_set(TAG, ESP_LOG_INFO);
    char *cmd_data;
    char *mark_pos;
    char *complete_pos = NULL;
    for (;;)
    {
        if (xQueueReceive(queue_cmd, &cmd_data, (portTickType)portMAX_DELAY))
        {
            ESP_LOGD(TAG, "cmd data [%d]: %s", strlen(cmd_data), cmd_data);
            mark_pos = strchr(cmd_data, '?');
            complete_pos = strstr(cmd_data, "\"Command Complete\"");
            if (mark_pos != NULL)
            {
                // The len_cmd is calculated from the original ? position
                size_t len_cmd = mark_pos - cmd_data;
                if (len_cmd <= 0)
                {
                    ESP_LOGW(TAG, "len_cmd is invalid: %d", len_cmd);
                }
                // We increase ? position (mark_pos) and decrease complete_pos
                // to skip the line feeds and carriage returns
                mark_pos += 4;
                complete_pos -= 2;
                size_t len_data = complete_pos - mark_pos;
                if (len_data <= 0)
                {
                    ESP_LOGW(TAG, "len_data is invalid: %d", len_data);
                }
                char *command = calloc(len_cmd + 1, sizeof(char));
                char *data = calloc(len_data + 1, sizeof(char));
                ESP_LOGD(TAG, "mark_pos: %p complete_pos: %p len_data: %d", mark_pos, complete_pos, len_data);
                memcpy(command, cmd_data, len_cmd);
                memcpy(data, mark_pos, len_data);
                // Parse the received command result
                parse_command(&gpsdo_state, command, data);
                ESP_LOGD(TAG, "Received command %s", command);
                ESP_LOGD(TAG, "Received data %s", data);
                free(command);
                free(data);
                command = NULL;
                data = NULL;
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
    uint8_t *tod_data;
    uint32_t gpsepoch, utctime;
    struct tm ts;
    time_t timestamp;

    esp_log_level_set(TAG, ESP_LOG_INFO);
    for (;;)
    {
        if (xQueueReceive(queue_tod, &tod_data, (portTickType)portMAX_DELAY))
        {
            // Process the timestamp from the message
            // Fields 27 to 30 contain a 32bit timestamp
            gpsepoch = (uint32_t)((tod_data[27] * (256 * 256 * 256)) + (tod_data[28] * (256 * 256)) + (tod_data[29] * (256)) + tod_data[30]);

            // We need the utc_offset to calculate the UTC time from GPS time
            gpsdo_state.utc_offset = tod_data[32];

            // Difference between GPS and UTC epoch
            // UTC Offset contains the leap seconds
            utctime = gpsepoch - gpsdo_state.utc_offset;
            utctime += 315964800;

            timestamp = utctime;
            ts = *localtime(&timestamp);
            strftime(gpsdo_state.date, GPSDO_STATE_DATE_SIZE, "%d %b %G", &ts);
            strftime(gpsdo_state.time, GPSDO_STATE_TIME_SIZE, "%X U", &ts);
            ESP_LOGD(TAG, "Parsed date: %s", gpsdo_state.date);
            ESP_LOGD(TAG, "Parsed time: %s", gpsdo_state.time);

            ESP_LOGD(TAG, "UTC Time: %d", utctime);

            gpsdo_state.week = (int)(gpsepoch / (7 * 24 * 60 * 60));
            ESP_LOGD(TAG, "GPS Week: %d", gpsdo_state.week);

            // tod_data[33]: 40=PPS validity?  41:phase settling  50:pps invalid?
            //           60:stable  62:stable, leap pending?
            // on power up: 41 -> 43 -> 63 -> 60/62 (62=leap pending?) Trimble
            // on power up: 41 -> 43 -> 60 -> 62 (62=leap pending?)    Trimble UCCM-P

            // tod_data[34]: 04=normal 0C=antenna open/shorted  06=normal?
            // on power up: 00 -> 04
            // disconnect antenna: 04 -> 0C
            // reconnect antenna   0C -> 04

            // tod_data[35]: 41=power up  4F=FFOM >0/settling/no antenna    45:FFOM 0,locked? - Trimble
            // on power up: 4F -> 45           Trimble UCCM-P
            // on power up: 41 -> 4F -> 45     Trimble UCCM
            // on antenna disconnect 45 -> 4F  Trimble
            // on antenna connect    4F -> 45

            // tod_data[36]: 80=have date?/normal  90:date invalid?/no antenna
            // power up 90 -> 80                   Trimble
            // disconnect antenna: 80 -> 90        Trimble UCCM-P
            // reconnect antenna:  90 -> 80        Trimble UCCM-P
            ESP_LOGD(TAG, "tod[33-36]: %d %d %d %d", tod_data[33], tod_data[34], tod_data[35], tod_data[36]);

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

static void uart_receive_cmd_task(void *pvParameters)
{
    static const char *TAG = "uart_receive_cmd";
    esp_log_level_set(TAG, ESP_LOG_INFO);

    uart_event_t event;

    int cmd_buffer_index = 0;

    char *prompt_pos;
    char *cmd_pos;
    bool cmd_detected = false;
    char *dtmp = malloc(CMD_BUFFER_SIZE);
    char *cmd_buffer = malloc(CMD_BUFFER_SIZE);
    bzero(dtmp, CMD_BUFFER_SIZE);
    bzero(cmd_buffer, CMD_BUFFER_SIZE);

    uart_flush_input(CMD_PORT_NUM);
    // xSemaphoreGive(can_send_cmd);

    for (;;)
    {
        if (xQueueReceive(queue_uart_cmd, (void *)&event, (portTickType)portMAX_DELAY) == pdTRUE)
        {
            bzero(dtmp, CMD_BUFFER_SIZE);
            switch (event.type)
            {
            /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
            case UART_DATA:
                uart_read_bytes(CMD_PORT_NUM, dtmp, event.size, 10 / portTICK_RATE_MS);

                // If the ? has been detected in the previous call, we just get
                // the rest of the data and notify the parsing function.
                if (cmd_detected)
                {
                    memcpy(&cmd_buffer[cmd_buffer_index], dtmp, event.size);
                    cmd_buffer_index += event.size;

                    prompt_pos = strstr(cmd_buffer, "UCCM> ");
                    if (prompt_pos != NULL)
                    {
                        int cmd_data_length = prompt_pos - cmd_buffer;
                        char *cmd_data = calloc(cmd_data_length + 1, sizeof(char));
                        memcpy(cmd_data, cmd_buffer, cmd_data_length);
                        ESP_LOGD(TAG, "Complex send [%d]: %s", cmd_data_length, cmd_data);
                        if (xQueueSendToBack(queue_cmd, &cmd_data, (portTickType)portMAX_DELAY) != pdPASS)
                        {
                            ESP_LOGE(TAG, "Error sending data to cmd queue");
                        }
                        // xSemaphoreGive(can_send_cmd);
                        // Zero out the buffer to prevent nastiness
                        cmd_buffer_index = 0;
                        bzero(cmd_buffer, TOD_BUFFER_SIZE);
                        cmd_detected = false;
                    }
                    break;
                }

                cmd_pos = strchr(dtmp, '?');
                prompt_pos = strstr(dtmp, "UCCM> ");
                if (cmd_pos != NULL)
                {
                    int cmd_length = cmd_pos - dtmp;
                    if ((cmd_length < 4) || (cmd_length > 18))
                    {
                        ESP_LOGW(TAG, "Illegal cmd_length: %d", cmd_length);
                        // xSemaphoreGive(can_send_cmd);
                        break;
                    }

                    if (prompt_pos != NULL)
                    {
                        int cmd_data_length = prompt_pos - dtmp;
                        char *cmd_data = calloc(cmd_data_length + 1, sizeof(char));
                        memcpy(cmd_data, dtmp, cmd_data_length);
                        ESP_LOGD(TAG, "Simple send: %s", cmd_data);
                        if (xQueueSendToBack(queue_cmd, &cmd_data, (portTickType)portMAX_DELAY) != pdPASS)
                        {
                            ESP_LOGE(TAG, "Error sending data to cmd queue");
                        }
                        // xSemaphoreGive(can_send_cmd);
                        break;
                    }
                    else
                    {
                        cmd_detected = true;
                        memcpy(&cmd_buffer[cmd_buffer_index], dtmp, event.size);
                        cmd_buffer_index = event.size;
                        break;
                    }
                }
                break;
            //Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGW(TAG, "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(CMD_PORT_NUM);
                xQueueReset(queue_uart_cmd);
                break;
            //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGW(TAG, "ring buffer full");
                // If buffer full happened, you should consider increasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(CMD_PORT_NUM);
                xQueueReset(queue_uart_cmd);
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
            //Others
            default:
                ESP_LOGE(TAG, "Undefined event: %d", event.type);
                break;
            }
        }
    }
    free(dtmp);
    free(cmd_buffer);
    vTaskDelete(NULL);
}

static void uart_receive_tod_task(void *pvParameters)
{
    static const char *TAG = "uart_receive_tod";

    uart_event_t event;

    int tod_buffer_index = 0;
    bool c5_detected = false;

    uint8_t *c5_pos;
    uint8_t *ca_pos;
    uint8_t *dtmp = calloc(TOD_BUFFER_SIZE, sizeof(uint8_t));
    uint8_t *tod_buffer = calloc(TOD_BUFFER_SIZE, sizeof(uint8_t));

    uart_flush_input(TOD_PORT_NUM);

    for (;;)
    {
        if (xQueueReceive(queue_uart_tod, (void *)&event, (portTickType)portMAX_DELAY))
        {
            switch (event.type)
            {
            /*We'd better handle data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
            case UART_DATA:
                uart_read_bytes(TOD_PORT_NUM, dtmp, event.size, portMAX_DELAY);

                // Check if the c5 string is in the buffer. If yes, wait another round and
                // then ship the data to the parsing function.'
                if (!c5_detected)
                {
                    c5_pos = memchr(dtmp, 0xc5, event.size);
                    if (c5_pos != NULL)
                    {
                        bzero(tod_buffer, TOD_BUFFER_SIZE);
                        c5_detected = true;
                        int data_length = event.size - (c5_pos - dtmp);
                        memcpy(&tod_buffer[0], c5_pos, data_length);
                        tod_buffer_index = data_length;
                        // ESP_LOGD(TAG, "C5 detected. tod_buffer_index: %d", tod_buffer_index);
                    }
                }
                // If the c5 has been detected in the previous call, we just get
                // the rest of the data and notify the parsing function.
                if (c5_detected)
                {
                    if (tod_buffer_index == TOD_PACKET_SIZE)
                    {
                        // Here notify the parsing task that data is ready
                        char *tod_data = calloc(TOD_PACKET_SIZE, sizeof(uint8_t));
                        memcpy(tod_data, tod_buffer, tod_buffer_index);
                        if (xQueueSendToBack(queue_tod, &tod_data, (portTickType)portMAX_DELAY) != pdPASS)
                        {
                            ESP_LOGE(TAG, "Error sending data to TOD queue");
                        }
                    }
                    else
                    {
                        // We add 2 to obtain the ca string itself
                        ca_pos = memchr(dtmp, 0xca, event.size);
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

                        ca_pos += 1;
                        memcpy(&tod_buffer[tod_buffer_index], dtmp, ca_pos - dtmp);
                        tod_buffer_index += ca_pos - dtmp;

                        ESP_LOGD(TAG, "Binary TOD [%d]", tod_buffer_index);
                        // for (int i = 0; i < tod_buffer_index; i++)
                        // {
                        //     ESP_LOGD(TAG, "Binary TOD [%d]: 0x%X", i, tod_buffer[i]);
                        // }

                        if (tod_buffer_index == TOD_PACKET_SIZE)
                        {
                            // Here notify the parsing task that data is ready
                            char *tod_data = calloc(TOD_PACKET_SIZE, sizeof(uint8_t));
                            memcpy(tod_data, tod_buffer, tod_buffer_index);
                            if (xQueueSendToBack(queue_tod, &tod_data, (portTickType)portMAX_DELAY) != pdPASS)
                            {
                                ESP_LOGE(TAG, "Error sending data to TOD queue");
                            }
                        }
                        else
                        {
                            ESP_LOGW(TAG, "Received TOD with incorrect packet size: %d", tod_buffer_index);
                        }
                    }

                    // Zero out the buffer to prevent nastiness
                    tod_buffer_index = 0;
                    bzero(tod_buffer, TOD_BUFFER_SIZE);
                    c5_detected = false;
                }
                break;
            //Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGW(TAG, "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(TOD_PORT_NUM);
                xQueueReset(queue_uart_tod);
                break;
            //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGW(TAG, "ring buffer full");
                // If buffer full happened, you should consider increasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(TOD_PORT_NUM);
                xQueueReset(queue_uart_tod);
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
    free(tod_buffer);
    vTaskDelete(NULL);
}

static void initialize_uart()
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
    // Command UART Initialization
    // Install UART driver, and get the queue.
    ESP_ERROR_CHECK(uart_driver_install(CMD_PORT_NUM, CMD_BUFFER_SIZE * 2, 0, 20, &queue_uart_cmd, 0));
    // Set configuration
    ESP_ERROR_CHECK(uart_param_config(CMD_PORT_NUM, &uart_config));
    // Set UART pins
    ESP_ERROR_CHECK(uart_set_pin(CMD_PORT_NUM, GPIO_NUM_17, GPIO_NUM_16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // TOD UART Initialization
    ESP_ERROR_CHECK(uart_param_config(TOD_PORT_NUM, &uart_config));
    //Set UART pins
    ESP_ERROR_CHECK(uart_set_pin(TOD_PORT_NUM, GPIO_NUM_26, GPIO_NUM_25, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    //Install UART driver, and get the queue.
    ESP_ERROR_CHECK(uart_driver_install(TOD_PORT_NUM, TOD_BUFFER_SIZE * 2, 0, 20, &queue_uart_tod, 0));
}

static void initialize_display()
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
    sprintf(holder, "DAC: %+7.4f %%", gpsdo_state.dac);
    u8g2_DrawStr(&u8g2, 0, 31, holder);
    sprintf(holder, "Phase: %+5.2E", gpsdo_state.phase);
    u8g2_DrawStr(&u8g2, 0, 39, holder);
    sprintf(holder, "PPS: %+7.4fns", gpsdo_state.pps);
    u8g2_DrawStr(&u8g2, 0, 47, holder);
    sprintf(holder, "FREQ DIFF: %5.2f", gpsdo_state.freq_diff);
    u8g2_DrawStr(&u8g2, 0, 55, holder);
    sprintf(holder, "TFOM: %d FFOM: %d", gpsdo_state.tfom, gpsdo_state.ffom);
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
    u8g2_DrawStr(&u8g2, 0, 15, gpsdo_state.time);
    u8g2_DrawStr(&u8g2, 0, 23, "Freq: N/A");
    u8g2_DrawStr(&u8g2, 0, 31, "GPSDO Status");
    sprintf(holder, "OUT: %.8s", gpsdo_state.status_output);
    u8g2_DrawStr(&u8g2, 0, 39, holder);
    sprintf(holder, "GPS: %.8s", gpsdo_state.status_gps);
    u8g2_DrawStr(&u8g2, 0, 47, holder);
    sprintf(holder, "Pos: %.8s", gpsdo_state.status_pos);
    u8g2_DrawStr(&u8g2, 0, 55, holder);
    sprintf(holder, "OPR: %.8s", gpsdo_state.status_opr);
    u8g2_DrawStr(&u8g2, 0, 63, holder);
    // Drawing of right side
    u8g2_DrawStr(&u8g2, 63, 15, gpsdo_state.date);
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
    char *holder = malloc(24 * sizeof(char));
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_6x12_tf);
    // Drawing of left side
    sprintf(holder, "Tracking: %d", gpsdo_state.satellite_trk);
    u8g2_DrawStr(&u8g2, 0, 7, holder);
    sprintf(holder, "Visible: %d", gpsdo_state.satellite_vis);
    u8g2_DrawStr(&u8g2, 0, 15, holder);
    u8g2_DrawStr(&u8g2, 0, 23, " PRN E1  AZ  C/N Sig.");
    // for (int i = 0; i < MIN(gpsdo_state.satellite_trk, 5); i++)
    for (int i = 0; i < 5; i++)
    {
        u8g2_DrawStr(&u8g2, 0, (31 + i * 8), " 00  00 000  000  00");
    }
    // u8g2_DrawStr(&u8g2, 0, 31, "  2  57  33   42  42");
    // u8g2_DrawStr(&u8g2, 0, 39, "  5  57 336   50  50");
    // u8g2_DrawStr(&u8g2, 0, 47, " 15  23 206   36  36");
    // u8g2_DrawStr(&u8g2, 0, 55, " 19  22 149  N/A  --");
    // u8g2_DrawStr(&u8g2, 0, 63, " 25  13 274  N/A  --");
    u8g2_SendBuffer(&u8g2);
}

void statScreen()
{
    char *holder = calloc(24, sizeof(char));
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_6x12_tf);
    // Drawing of left side
    u8g2_DrawStr(&u8g2, 0, 7, gpsdo_state.time);
    u8g2_DrawStr(&u8g2, 0, 15, gpsdo_state.date);
    sprintf(holder, "Week: %5d", gpsdo_state.week);
    u8g2_DrawStr(&u8g2, 0, 23, holder);
    u8g2_DrawStr(&u8g2, 0, 31, "Tow: 000000");
    sprintf(holder, "UTF ofs: %3d", gpsdo_state.utc_offset);
    u8g2_DrawStr(&u8g2, 0, 39, holder);
    sprintf(holder, "Alt: %+6.3f m", gpsdo_state.altitude);
    u8g2_DrawStr(&u8g2, 0, 47, holder);
    sprintf(holder, "Lat: %+2.7f", gpsdo_state.latitude);
    u8g2_DrawStr(&u8g2, 0, 55, holder);
    sprintf(holder, "Lon: %+2.7f", gpsdo_state.longitude);
    u8g2_DrawStr(&u8g2, 0, 63, holder);
    // Drawing of right side
    u8g2_DrawStr(&u8g2, 81, 7, "GPS STAT");
    sprintf(holder, "OUT:%.4s", gpsdo_state.status_output);
    u8g2_DrawStr(&u8g2, 81, 15, holder);
    sprintf(holder, "GPS:%.4s", gpsdo_state.status_gps);
    u8g2_DrawStr(&u8g2, 81, 23, holder);
    sprintf(holder, "Pos:%.4s", gpsdo_state.status_pos);
    u8g2_DrawStr(&u8g2, 81, 31, holder);
    u8g2_DrawStr(&u8g2, 81, 39, "Stable");
    u8g2_SendBuffer(&u8g2);
}
