#ifndef MAIN_H_
#define MAIN_H_

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
    int status_output;
    int status_gps;
    int status_pos;
    int status_opr;
    char alarm_hw[11];
    char alarm_op[11];
    int week;
    int tow;
    int utc_offset;
    float altitude;
    float latitude;
    float longitude;
    gps_satellite_t satellites[24];
} gpsdo_state_t;

#endif