#ifndef MODBUS_COMMON_H
#define MODBUS_COMMON_H
#include <stdint.h>

#ifndef NULL
#define NULL ((void*)0)
#endif

static inline uint16_t U16_MSBFromStream(uint8_t* data) {
  return (data[0] << 8) | data[1];
}

static inline uint16_t U16_LSBFromStream(uint8_t* data) {
  return data[0] | (data[1] << 8);
}

static inline void U16_MSB2Stream(uint16_t val, uint8_t* data) {
  *data = (val & 0xff00) >> 8;
  *(++data) = val & 0x00ff;
}

static inline void U16_LSB2Stream(uint16_t val, uint8_t* data) {
  *data = val & 0x00ff;
  *(++data) = (val & 0xff00) >> 8;
}

static inline uint16_t nearestMultipleOf8(uint16_t val) {
  return (val + 7) & ~7;
}
////////////////////////////////////////////////////////////////////////////

#endif  // MODBUS_COMMON_H
