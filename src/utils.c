#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "main.h"
#include "utils.h"
#include "esp_log.h"

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

void parse_command(gpsdo_state_t *gpsdo_status, char *command, char *data)
{
    static const char *TAG = "parse_command";

    char *data_ptr = data;
    // This is a dirty hack to have a switch for a string.
    // Each string maps into a different long number
    ESP_LOGD(TAG, "%s: %lu", command, hash(command));
    switch (hash(command))
    {
    case 2088064778: /* *IDN? */
        strncpy(gpsdo_status->manufacturer, strsep(&data_ptr, ","), 10);
        strncpy(gpsdo_status->model, strsep(&data_ptr, ","), 10);
        strncpy(gpsdo_status->serial_number, strsep(&data_ptr, ","), 10);
        strncpy(gpsdo_status->version, strsep(&data_ptr, ","), 10);
        ESP_LOGD(TAG, "Brand: %s", gpsdo_status->manufacturer);
        ESP_LOGD(TAG, "Model: %s", gpsdo_status->model);
        ESP_LOGD(TAG, "Serial: %s", gpsdo_status->serial_number);
        ESP_LOGD(TAG, "Version: %s", gpsdo_status->version);
        break;
    case 4166190878: /* ALARM:HARD? */
        strncpy(gpsdo_status->alarm_hw, data, 10);
        ESP_LOGD(TAG, "H/W Alarm: %s", gpsdo_status->alarm_hw);
        break;
    case 4166458357: /* ALARM:OPER? */
        strncpy(gpsdo_status->alarm_op, data, 10);
        ESP_LOGD(TAG, "Oper Alarm: %s", gpsdo_status->alarm_op);
        break;
    case 2002553166: /* DIAG:LOOP? OCXO is loop Frequency offset*/
        /*
        OCXO : +1.706E-8
        EXT : Unavailable
        */
        // ESP_LOGD(TAG, "Data: %s", data);
        break;
    case 1674576848: /* DIAG:ROSC:EFC:REL? AC in percentage */
    {
        gpsdo_status->dac = atof(data);
        ESP_LOGD(TAG, "DAC: %f", gpsdo_status->dac);
        break;
    }
    case 3720921287: /* DIAG:ROSC:EFC:DATA? Unknown*/
        /* +1.700E-8 */
        // ESP_LOGD(TAG, "Data: %s", data);
        break;
    case 499271643: /* GPS:POS? */
    {
        double sign = 0.0;
        double lat = 0.0;
        double lon = 0.0;
        double alt = 0.0;
        char letter;
        // Parse latitude
        letter = strsep(&data_ptr, ",")[0];
        sign = 1.0;
        if (letter == 'S')
            sign = -1.0;
        else
            sign = 1.0;
        lat = atof(strsep(&data_ptr, ","));
        lat += atof(strsep(&data_ptr, ",")) / 60.0;
        lat += atof(strsep(&data_ptr, ",")) / 3600.0;
        lat *= sign;

        // Parse longitude
        letter = strsep(&data_ptr, ",")[0];
        if (letter == 'W')
            sign = -1.0;
        else
            sign = 1.0;
        lon = atof(strsep(&data_ptr, ","));
        lon += atof(strsep(&data_ptr, ",")) / 60.0;
        lon += atof(strsep(&data_ptr, ",")) / 3600.0;
        lon *= sign;

        // Parse height
        alt = atof(strsep(&data_ptr, ","));

        gpsdo_status->latitude = lat;
        gpsdo_status->longitude = lon;
        gpsdo_status->altitude = alt;

        ESP_LOGD(TAG, "Lat: %f", lat);
        ESP_LOGD(TAG, "Lon: %f", lon);
        ESP_LOGD(TAG, "Alt: %f", alt);
        break;
    }
    case 26313482: /* LED:GPSL? */
        /* Locked */
        strncpy(gpsdo_status->status_gps, data, 8);
        ESP_LOGD(TAG, "Data: %s", gpsdo_status->status_gps);
        break;
    case 1465923843: /* OUTP:STAT? */
        /* Normal */
        strncpy(gpsdo_status->status_output, data, 8);
        ESP_LOGD(TAG, "Data: %s", gpsdo_status->status_output);
        break;
    case 1470096006: /* PULLINRANGE? */
        /* Pull-in Range : [30 ppb] */
        break;
    case 3995171108: /* SYNC:FFOM? */
        /* PLL stabilized */
        break;
    /* SYNC:TFOM? */
    case 3995677467: /* SYNC:TINT? */
        /* -7.229E-10 */
        ESP_LOGD(TAG, "Data: %s", data);
        break;
    case 2528360014: /* SYST:STAT? */
        parse_status(gpsdo_status, data);
        break;
    default:
        ESP_LOGI(TAG, "Unhashed command: %s", command);
        ESP_LOGI(TAG, "Hash: %lu", hash(command));
        ESP_LOGI(TAG, "Data: %s", data);
        break;
    }
}

void parse_status(gpsdo_state_t *gpsdo_status, char *data)
{
    static const char *TAG = "parse_status";

    int counter = 0;

    char *found;
    char *data_ptr = data;
    while ((found = strsep(&data_ptr, "\n")) != NULL)
    {
        switch (counter++)
        {
        case 5:
            /* TFOM     0            FFOM      0 */
            {
                char *start;
                char *end;
                char *string;
                int length = 0;
                start = strstr(found, "TFOM") + 7;
                end = start + 6;
                length = end - start;
                string = calloc(length + 1, sizeof(char));
                if ((start != NULL) && (end != NULL) && (length > 0))
                {
                    memcpy(string, start, length);
                    gpsdo_status->tfom = atoi(string);
                    ESP_LOGD(TAG, "TFOM: %d", gpsdo_status->tfom);
                }

                start = strstr(found, "FFOM") + 5;
                end = start + 6;
                length = end - start;
                string = calloc(length + 1, sizeof(char));
                if ((start != NULL) && (end != NULL) && (length > 0))
                {
                    memcpy(string, start, length);
                    gpsdo_status->ffom = atoi(string);
                    ESP_LOGD(TAG, "FFOM: %d", gpsdo_status->ffom);
                }
                continue;
            }

        case 8:
            /* >>GPS :     [phase : -4.714E-10] */
            {
                char *start = strstr(found, "phase : ") + 8;
                char *end = strchr(found, ']');
                int length = end - start;
                char *string = calloc(length + 1, sizeof(char));
                if ((start != NULL) && (end != NULL) && (length > 0))
                {
                    memcpy(string, start, length);
                    gpsdo_status->phase = atof(string);
                    ESP_LOGD(TAG, "Phase: %3.3E", gpsdo_status->phase);
                }
                else
                {
                    ESP_LOGD(TAG, "Start: %p End: %p Length: %d", start, end, length);
                }
                continue;
            }
        case 9:
            /* ACQUISITION ................................................ [ GPS 1PPS Valid ] */
            continue;
        case 10:
            /* Tracking:  7 ___   Not Tracking:  5 _______   Time ____________________________ */
            {
                char *start;
                char *end;
                char *string;
                int length = 0;
                start = strstr(found, "Tracking:") + 9;
                end = strchr(found, '_');
                length = end - start;
                string = calloc(length + 1, sizeof(char));
                if ((start != NULL) && (end != NULL) && (length > 0))
                {
                    memcpy(string, start, length);
                    gpsdo_status->satellite_trk = atoi(string);
                    ESP_LOGD(TAG, "Tracking: %d", gpsdo_status->satellite_trk);
                }

                start = strstr(end, "Tracking:") + 9;
                end = strchr(start, '_');
                length = end - start;
                string = calloc(length + 1, sizeof(char));
                if ((start != NULL) && (end != NULL) && (length > 0))
                {
                    memcpy(string, start, length);
                    gpsdo_status->satellite_vis = gpsdo_status->satellite_trk + atoi(string);
                    ESP_LOGD(TAG, "Visible: %d", gpsdo_status->satellite_vis);
                }

                continue;
            }
        case 11:
            /* PRN  El  AZ  CNO   PRN  El  Az                GPS      09:23:09     13 OCT 2021 */
            continue;
        case 12:
        case 13:
        case 14:
        case 15:
        case 16:
        case 17:
        case 18:
            /* These are the lines possibly containing GPS info */
            /* 1  63 139  50      6   6 304                GPS      Synchronized to UTC */
            continue;
        case 24:
            /* ELEV MASK  5 deg                              ANT V=5.112V, I=24.400mA */
            continue;
        case 26:
            /* Temp = 37.000 / NONE */
            {
                char *start = strstr(found, "Temp =") + 6;
                char *end = strchr(found, '/');
                int length = end - start;
                char *string = calloc(length + 1, sizeof(char));
                if ((start != NULL) && (end != NULL) && (length > 0))
                {
                    memcpy(string, start, length);
                    gpsdo_status->temperature = atof(string);
                    ESP_LOGD(TAG, "Temperature: %2.3f", gpsdo_status->temperature);
                }
                continue;
            }
        default:
            ESP_LOGD(TAG, "Xableta");
            continue;
        }
    }
}
