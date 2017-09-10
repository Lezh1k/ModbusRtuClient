#ifndef MODBUS_RTU_CLIENT_H
#define MODBUS_RTU_CLIENT_H

#include <stdint.h>

////////////////////////////////////////////////////////////////////////////
typedef enum mb_exception_code {
  mbec_OK = 0,
  mbec_illegal_function = 0x01,
  mbec_illegal_data_address = 0x02,
  mbec_illegal_data_value = 0x03,
  mbec_service_device_failure = 0x04,
  mbec_acknowledge = 0x05,
  mbec_server_device_busy = 0x06,
  mbec_memory_parity_error = 0x08,
  mbec_gateway_path_unavailable = 0x0a,
  mbec_gateway_target_device_failed_to_respond = 0x0b,
  mbec_heap_error = 0x0c,  // hope nobody uses this
} mbec_exception_code_t;
//////////////////////////////////////////////////////////////////////////

typedef uint16_t mb_register;

typedef enum mb_func_code {
  /*STANDARD FUNCTIONS*/
  /*rw coils*/
  mbfc_read_discrete_input = 0x02,
  mbfc_read_coils = 0x01,
  mbfc_write_single_coil = 0x05,
  mbfc_write_multiple_coils = 0x0f,
  /*rw registers*/
  mbfc_read_input_registers = 0x04,
  mbfc_read_holding_registers = 0x03,
  mbfc_write_single_register = 0x06,
  mbfc_write_multiple_registers = 0x10,
  mbfc_read_write_multiple_registers = 0x17,
  mbfc_mask_write_registers = 0x16,
  /*r fifo*/
  mbfc_read_fifo = 0x18,
  /*diagnostic*/
  mbfc_read_file_record = 0x14,
  mbfc_write_file_record = 0x15,
  mbfc_read_exception_status = 0x07,
  mbfc_diagnostic = 0x08,
  mbfc_get_com_event_counter = 0x0b,
  mbfc_get_com_event_log = 0x0c,
  /*misc*/
  mbfc_report_device_id = 0x11,
  mbfc_encapsulate_tp_info = 0x2b,

  /*USER DEFINED FUNCTIONS 65-72, 100-110*/
} mb_func_code_t;
////////////////////////////////////////////////////////////////////////////

typedef enum mb_adu_size { mbaz_rs485 = 256, mbaz_tcp = 260 } mb_adu_size_t;
//////////////////////////////////////////////////////////////////////////

typedef struct mb_dev_bit_mapping {
  uint16_t start_addr;
  uint16_t end_addr;
  uint8_t* real_addr;
} mb_dev_bit_mapping_t;

typedef struct mb_dev_registers_mapping {
  uint16_t start_addr;
  uint16_t end_addr;
  uint16_t* real_addr;
} mb_dev_registers_mapping_t;

typedef struct mb_client_device {
  uint8_t address;                                   // ID [1..247].
  mb_dev_bit_mapping_t input_discrete_map;           // read bits
  mb_dev_bit_mapping_t coils_map;                    // read/write bits
  mb_dev_registers_mapping_t input_registers_map;    // read registers
  mb_dev_registers_mapping_t holding_registers_map;  // read/write registers
  void (*tp_send)(uint8_t* data, uint16_t len);      // transport send
} mb_client_device_t;

void mb_init(mb_client_device_t* dev);
uint16_t mb_handle_request(uint8_t* data, uint16_t data_len);

#endif  // MODBUS_RTU_CLIENT_H
