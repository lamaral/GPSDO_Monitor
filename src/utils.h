#ifndef UTILS_H_
#define UTILS_H_

uint32_t atohex(char *s);
unsigned long hash(const char *str);
void parse_command(gpsdo_state_t *gpsdo_status, char *command, char *data);
void parse_status(gpsdo_state_t *gpsdo_status, char *data);

#endif
