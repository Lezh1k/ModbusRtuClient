#ifndef MODBUS_COMMON_H
#define MODBUS_COMMON_H
#include <stdint.h>

#ifndef NULL
#define NULL ((void*)0)
#endif

static inline uint16_t U16_MSB_from_stream(uint8_t* data) {
  return (data[0] << 8) | data[1];
}

static inline uint16_t U16_LSB_from_stream(uint8_t* data) {
  return data[0] | (data[1] << 8);
}

static inline void U16_MSB_to_stream(uint16_t val, uint8_t* data) {
  *data = (val & 0xff00) >> 8;
  *(++data) = val & 0x00ff;
}

static inline void U16_LSB_to_stream(uint16_t val, uint8_t* data) {
  *data = val & 0x00ff;
  *(++data) = (val & 0xff00) >> 8;
}

static inline uint16_t nearest_8_multiple(uint16_t val) {
  return (val + 7) & ~7;
}
////////////////////////////////////////////////////////////////////////////

typedef struct read_bits_arg {
  uint16_t address;
  uint16_t quantity;
} read_bits_arg_t;
//

typedef struct read_registers_arg {
  uint16_t address;
  uint16_t quantity;
} read_registers_arg_t;
//

typedef enum coin_state {
  cs_on = 0xff00,
  cs_off = 0x00ff
} coin_state_t;
//

typedef struct write_single_coil_arg {
  uint16_t address;
  uint16_t coil_state;
} write_single_coil_arg_t;
//

typedef struct write_single_register_arg {
  uint16_t address;
  uint16_t data;
} write_single_register_arg_t;
//

typedef struct write_multiple_coils_arg {
  uint16_t address;
  uint16_t quantity;
  uint8_t byte_count;
  uint8_t *data;
} write_multiple_coils_arg_t;
//

typedef struct write_multiple_registers_arg {
  uint16_t address;
  uint16_t quantity;
  uint8_t byte_count;
  uint8_t* data;
} write_multiple_registers_arg_t;
//

typedef struct mask_write_register_arg {
  uint16_t address;
  uint16_t and_mask;
  uint16_t or_mask;
} mask_write_register_arg_t;
////////////////////////////////////////////////////////////////////////////

typedef enum diagnostics_sub_code {
  dsc_return_query_data = 0,
  dsc_restart_communications_option,
  dsc_return_diagnostic_register,
  dsc_change_adcii_input_delimiter,
  dsc_force_listen_only_mode,
  dsc_reserved05,
  dsc_reserved06,
  dsc_reserved07,
  dsc_reserved08,
  dsc_reserved09,
  dsc_clean_counter_and_diagnostic_registers,
  dsc_return_bus_messages_count,
  dsc_return_bus_communication_error_count,
  dsc_return_bus_exception_error_count,
  dsc_return_server_messages_count,
  dsc_return_server_no_response_count,
  dsc_return_server_NAK_count,
  dsc_return_server_busy_count,
  dsc_return_bus_character_overrun_count,
  dsc_reserved19,
  dsc_clear_overrun_counter_and_flag,
  dsc_reserved
} diagnostics_sub_code_t;
//

typedef struct diagnostics_arg {
  uint16_t sub_function;
} diagnostics_arg_t;
//

#endif // MODBUS_COMMON_H
