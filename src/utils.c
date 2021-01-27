#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "main.h"
#include "utils.h"
#include "esp_log.h"

// Used for debug output
static const char *TAG = "utils";

uint32_t atohex(char *s)
{
    uint32_t val;

    // return 32-bit integer value of a hex ascii string (can have leading 0x)

    if (s == 0)
        return 0;
    if ((s[0] == '0') && (s[1] == 'x'))
        s += 2;
    else if ((s[0] == '0') && (s[1] == 'X'))
        s += 2;

    val = 0;
    sscanf(s, "%x", &val);
    return val;
}

unsigned long hash(const char *str)
{
    unsigned long hash = 5381;
    int c;

    while ((c = *str++))
        hash = ((hash << 5) + hash) + c;
    return hash;
}

void parse_command(gpsdo_state_t gpsdo_status, char *command, char *data)
{
    char *data_ptr = data;
    // This is a dirty hack to have a switch for a string.
    // Each string maps into a different long number
    switch (hash(command))
    {
    case 186661001: /* *IDN? */
        strcpy(gpsdo_status.manufacturer, strsep(&data_ptr, ","));
        strcpy(gpsdo_status.model, strsep(&data_ptr, ","));
        strcpy(gpsdo_status.serial_number, strsep(&data_ptr, ","));
        strcpy(gpsdo_status.version, strsep(&data_ptr, ","));
        ESP_LOGD(TAG, "Brand: %s", gpsdo_status.manufacturer);
        ESP_LOGD(TAG, "Model: %s", gpsdo_status.model);
        ESP_LOGD(TAG, "Serial: %s", gpsdo_status.serial_number);
        ESP_LOGD(TAG, "Version: %s", gpsdo_status.version);
        break;
    case 4019269066: /* ALARM:HARD? */
        break;
    case 4028095873: /* ALARM:OPER? */
        break;
    case 1659745101: /* DIAG:LOOP? */
        break;
    case 3721428495: /* DIAG:ROSC:EFC:REL? */
        break;
    case 2531318246: /* DIAG:ROSC:EFC:DATA? */
        break;
    case 3591062394: /* GPS:POS? */
        break;
    case 868344969: /* LED:GPSL? */
        break;
    case 1130846626: /* OUTP:STAT? */
        break;
    case 1268528005: /* PULLINRANGE? */
        break;
    case 3008337594: /* SYNC:TINT? */
        break;
    case 1831501901: /* SYST:STAT? */
        break;
    default:
        ESP_LOGI(TAG, "Unhashed command: %s", command);
        ESP_LOGI(TAG, "Hash: %lu", hash(command));
        ESP_LOGI(TAG, "Data: %s", data);
        break;
    }
}
