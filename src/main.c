#include <stdio.h>

#include "commons.h"
#include "heap_memory.h"
#include "modbus_rtu_client.h"

void
send_stub(uint8_t *data, uint16_t len) {
  while(len--) {
    printf("%x ", *data);
    ++data;
  }
  printf("\n");
}
////////////////////////////////////////////////////////////////////////////

int main() {

  uint8_t input_discrete_real[24] =
  { 0x00, 0x14, 0x22, 0x20, 0x00, 0x00, 0x00, 0x00 };
  //0001 0100 0010 0010 0010 0000
  uint8_t coils_real[24] =
  { 0x00, 0x14, 0x22, 0x20, 0x00, 0x00, 0x00, 0x00 };

  uint16_t input_registers_real[24] =
  { 0x0006, 0x0005, 0x0004,0x0006, 0x0005, 0x0004,
    0x0006, 0x0005, 0x0004,0x0006, 0x0005, 0x0004,
    0x0006, 0x0005, 0x0004,0x0006, 0x0005, 0x0004,
    0x0006, 0x0005, 0x0004,0x0006, 0x0005, 0x0004 };

  uint16_t holding_registers_real[24] =
  { 0x0006, 0x0005, 0x0004,0x0006, 0x0005, 0x0004,
    0x0006, 0x0005, 0x0004,0x0006, 0x0005, 0x0004,
    0x0006, 0x0005, 0x0004,0x0006, 0x0005, 0x0004,
    0x0006, 0x0005, 0x0004,0x0006, 0x0005, 0x0004 };

  mb_client_device_t dev = {
    .address = 1,  // ID [1..247].

    .input_discrete_map.start_addr = 0,  // r bits
    .input_discrete_map.end_addr = sizeof(input_discrete_real),
    .input_discrete_map.real_addr = input_discrete_real,

    .coils_map.start_addr = 0,  // rw bits
    .coils_map.end_addr = sizeof(coils_real),
    .coils_map.real_addr = coils_real,

    .input_registers_map.start_addr = 0,  // r registers
    .input_registers_map.end_addr = sizeof(input_registers_real),
    .input_registers_map.real_addr = input_registers_real,

    .holding_registers_map.start_addr = 0,  // rw registers
    .holding_registers_map.end_addr = sizeof(holding_registers_real),
    .holding_registers_map.real_addr = holding_registers_real,
    .tp_send = send_stub
  };

  uint8_t read_coils_arr[] = {
    0x04, 0x01, 0x00, 0x0a,
    0x00, 0x0d, 0xdd, 0x98
  };

  uint8_t read_input_discrete_arr[] = {
    0x04, 0x02, 0x00, 0x0a,
    0x00, 0x0d, 0x99, 0x98
  };

  uint8_t read_holding_registers_arr[] = {
    0x01, 0x03, 0x00, 0x00,
    0x00, 0x02, 0xc4, 0x0b
  };

  uint8_t read_input_registers_arr[] = {
    0x01, 0x04, 0x00, 0x00,
    0x00, 0x02, 0x71, 0xcb
  };

  uint8_t write_single_coil_arr[] = {
    0x11, 0x05, 0x00, 0xac,
    0xff, 0x00, 0x4e, 0x8b
  };

  uint8_t write_multiple_coils_arr[] = {
    0x04, 0x0F, 0x00, 0x20, 0x00, 0x10,
    0x02, 0xCD, 0x01, 0x4f, 0x40
  };

  uint8_t write_multiple_registers_arr[] = {
    0x11, 0x10, 0x00, 0x01, 0x00, 0x02, 0x04,
    0x00, 0x0a, 0x01, 0x02, 0xc6, 0xf0
  };

  uint8_t request_device_id_arr[] = {
    0x11, 0x11, 0xcd, 0xec
  };

  uint8_t write_single_register_arr[] = {
    0x11, 0x06, 0x00, 0x01, 0x00, 0x03, 0x9a, 0x9b
  };

  hm_init();

  mb_init(&dev);

  dev.address = 4;
  printf("read coils : ");
  mb_handle_request(read_coils_arr, sizeof(read_coils_arr));
  printf("read input discrete : ");
  mb_handle_request(read_input_discrete_arr, sizeof(read_input_discrete_arr));

  dev.address = 1;
  printf("read holding registers : ");
  mb_handle_request(read_holding_registers_arr, sizeof(read_holding_registers_arr));
  printf("read input registers : ");
  mb_handle_request(read_input_registers_arr, sizeof(read_input_registers_arr));
  dev.address = 0x11;
  printf("write single coil : ");
  mb_handle_request(write_single_coil_arr, sizeof(write_single_coil_arr));

  dev.address = 4;
  printf("write multiple coils : ");
  mb_handle_request(write_multiple_coils_arr, sizeof(write_multiple_coils_arr));

  dev.address = 0x11;
  printf("write multiple registers : ");
  mb_handle_request(write_multiple_registers_arr, sizeof(write_multiple_registers_arr));
  printf("request device id : ");
  mb_handle_request(request_device_id_arr, sizeof(request_device_id_arr));
  printf("write single register : ");
  mb_handle_request(write_single_register_arr, sizeof(write_single_register_arr));

  return 0;
}
