#ifndef MAIN_H_
#define MAIN_H_

#define GPSDO_STATE_DATE_SIZE 12
#define GPSDO_STATE_TIME_SIZE 13

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#define MIN(x, y) (((x) < (y)) ? (x) : (y))

void initialize_uccm();
void statScreen();
void monitorScreen();
void uccmDataScreen();
void satellitesScreen();
void splashPage();

// Struct that holds the GPS satellite data
typedef struct
{
    int prn;
    int e1;
    int az;
    int cn;
    int sig;
} gps_satellite_t;

// Struct that holds the status data for the GPSDO
typedef struct
{
    char manufacturer[11];
    char model[11];
    char serial_number[11];
    char version[11];
    float temperature;
    float dac;
    float phase;
    float pps;
    float freq_diff;
    int tfom;
    int ffom;
    char status_output[11];
    char status_gps[11];
    char status_pos[11];
    char status_opr[11];
    char alarm_hw[11];
    char alarm_op[11];
    int week;
    int tow;
    int utc_offset;
    char date[GPSDO_STATE_DATE_SIZE];
    char time[GPSDO_STATE_TIME_SIZE];
    float altitude;
    float latitude;
    float longitude;
    int satellite_trk;
    int satellite_vis;
    gps_satellite_t satellites[24];
} gpsdo_state_t;

#endif