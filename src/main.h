#ifndef MAIN_H_
#define MAIN_H_

static void uart_receive_task(void *pvParameters);
static void tod_parse_task(void *pvParameters);
static void cmd_parse_task(void *pvParameters);
static void update_display_task(void *pvParameters);
static void uart_init();
static void display_init();
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
    int alarm_hw;
    int alarm_op;
    int week;
    int tow;
    int utc_offset;
    float altitude;
    float latitude;
    float longitude;
    gps_satellite_t satellites[24];
} gpsdo_state_t;

#endif