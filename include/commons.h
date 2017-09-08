#ifndef COMMONS_H
#define COMMONS_H

#include <stdint.h>

#define UNUSED_ARG(x) ((void)x)
#define F_CPU 8000000

char* uint16_to_str(char* buff, uint16_t val, uint8_t buff_len);
char* int16_to_str(char* buff, int16_t val, uint16_t buff_len);
uint16_t crc16(uint8_t* msg, uint16_t len);
void print_binary(uint8_t val);

#endif  // COMMONS_H
