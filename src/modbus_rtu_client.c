#include "commons.h"
#include "modbus_rtu_client.h"
#include "heap_memory.h"
#include "modbus_common.h"

#include <stdio.h>

#pragma pack(push)
#pragma pack(1)
typedef struct mb_adu {
  uint8_t addr;
  uint8_t fc;
  uint8_t* data;
  uint8_t data_len;
  uint16_t crc; //should be little-endian
} mb_adu_t;
#pragma pack(pop)

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

enum {
  coin_state_on = 0xff00,
  coin_state_off = 0x0000
};

static inline uint16_t adu_buffer_len(mb_adu_t* adu) {
  return adu->data_len +
      sizeof(mb_adu_t) -
      sizeof(adu->data) -
      sizeof(adu->data_len);
}
////////////////////////////////////////////////////////////////////////////

typedef struct mb_counters {
  uint16_t bus_msg;       //cpt1 bus message count
  uint16_t bus_com_err;   //cpt2 bus communication error count
  uint16_t exc_err;       //cpt3 slave exception error count
  uint16_t slave_msg;     //cpt4 slave message count
  uint16_t slave_no_resp; //cpt5 return slave no response count
  uint16_t slave_NAK;     //cpt6 return slave NAK count
  uint16_t slave_busy;    //cpt7 return slave busy count
  uint16_t bus_char_overrrun; //cpt8 return bus character overrun count
} mb_counters_t;
//////////////////////////////////////////////////////////////////////////

typedef struct mb_request_handler {
  uint8_t   fc;
  uint16_t  fc_validation_result;
  uint16_t  (*pf_check_address)(mb_adu_t *adu);
  uint16_t  (*pf_validate_data_value)(mb_adu_t *adu);
  uint16_t  (*pf_execute_function)(mb_adu_t *adu);
} mb_request_handler_t;

static mb_adu_t *adu_from_stream(uint8_t *data, uint16_t len);
static uint8_t *adu_serialize(mb_adu_t *adu); //create on heap
static uint16_t mb_send_response(mb_adu_t *adu);
static void mb_send_exc_response(mbec_exception_code_t exc_code, mb_adu_t *adu);
static mb_request_handler_t* mb_validate_function_code(mb_adu_t* adu);

static void handle_broadcast_message(uint8_t *data, uint16_t len);
//////////////////////////////////////////////////////////////////////////

/*diagnostic handlers*/
static uint16_t diag_return_query_data(mb_adu_t *adu);
static uint16_t diag_restart_communications_option(mb_adu_t *adu);
static uint16_t diag_return_diagnostic_register(mb_adu_t *adu);
static uint16_t diag_change_adcii_input_delimiter(mb_adu_t *adu);
static uint16_t diag_force_listen_only_mode(mb_adu_t *adu);
static uint16_t diag_clean_counter_and_diagnostic_registers(mb_adu_t *adu);
static uint16_t diag_return_bus_messages_count(mb_adu_t *adu);
static uint16_t diag_return_bus_communication_error_count(mb_adu_t *adu);
static uint16_t diag_return_bus_exception_error_count(mb_adu_t *adu);
static uint16_t diag_return_server_messages_count(mb_adu_t *adu);
static uint16_t diag_return_server_no_response_count(mb_adu_t *adu);
static uint16_t diag_return_server_NAK_count(mb_adu_t *adu);
static uint16_t diag_return_server_busy_count(mb_adu_t *adu);
static uint16_t diag_return_bus_character_overrun_count(mb_adu_t *adu);
static uint16_t diag_clear_overrun_counter_and_flag(mb_adu_t *adu);

typedef uint16_t (*pf_diagnostic_data_t)(mb_adu_t *adu);
static pf_diagnostic_data_t diagnostic_data_handlers[] = {
  diag_return_query_data, diag_restart_communications_option, diag_return_diagnostic_register,
  diag_change_adcii_input_delimiter, diag_force_listen_only_mode,
  NULL, NULL, NULL, NULL, NULL,
  diag_clean_counter_and_diagnostic_registers, diag_return_bus_messages_count,
  diag_return_bus_communication_error_count, diag_return_bus_exception_error_count,
  diag_return_server_messages_count, diag_return_server_no_response_count,
  diag_return_server_NAK_count, diag_return_server_busy_count,
  diag_return_bus_character_overrun_count, NULL,
  diag_clear_overrun_counter_and_flag
};
/*diagnostic handlers END*/
//////////////////////////////////////////////////////////////////////////

/*check address functions*/
static uint16_t check_discrete_input_address(mb_adu_t *adu);
static uint16_t check_coils_address(mb_adu_t *adu);
static uint16_t check_input_registers_address(mb_adu_t *adu);
static uint16_t check_holding_registers_address(mb_adu_t *adu);
static uint16_t check_address_and_return_ok(); //this is for action functions (not read/write)
/*check address functions END*/

/*check data functions*/
static uint16_t check_read_discrete_input_data(mb_adu_t *adu);
static uint16_t check_read_coils_data(mb_adu_t *adu);
static uint16_t check_write_single_coil_data(mb_adu_t *adu);
static uint16_t check_write_multiple_coils_data(mb_adu_t *adu);
static uint16_t check_read_input_registers_data(mb_adu_t *adu);
static uint16_t check_write_single_register_data(mb_adu_t *adu);
static uint16_t check_read_holding_registers_data(mb_adu_t *adu);
static uint16_t check_write_multiple_registers_data(mb_adu_t *adu);
static uint16_t check_read_write_multiple_registers_data(mb_adu_t *adu);
static uint16_t check_mask_write_registers_data(mb_adu_t *adu);
static uint16_t check_read_fifo_data(mb_adu_t *adu);
static uint16_t check_read_file_record_data(mb_adu_t *adu);
static uint16_t check_write_file_record_data(mb_adu_t *adu);
static uint16_t check_read_exception_status_data(mb_adu_t *adu);
static uint16_t check_diagnostic_data(mb_adu_t *adu);
static uint16_t check_get_com_event_counter_data(mb_adu_t *adu);
static uint16_t check_get_com_event_log_data(mb_adu_t *adu);
static uint16_t check_report_device_id_data(mb_adu_t *adu);
static uint16_t check_encapsulate_tp_info_data(mb_adu_t *adu);
/*check data functions end*/

/*STANDARD FUNCTIONS HANDLERS*/

static uint16_t mb_read_bits(mb_adu_t *adu, uint8_t *real_addr);
static uint16_t execute_read_discrete_inputs(mb_adu_t *adu);
static uint16_t execute_read_coils(mb_adu_t *adu);

static uint16_t execute_write_single_coil(mb_adu_t *adu);
static uint16_t execute_write_multiple_coils(mb_adu_t *adu);

static uint16_t mb_read_registers(mb_adu_t *adu, uint16_t *real_addr);
static uint16_t execute_read_input_registers(mb_adu_t *adu);
static uint16_t execute_read_holding_registers(mb_adu_t *adu);
static uint16_t execute_write_single_register(mb_adu_t *adu);
static uint16_t execute_write_multiple_registers(mb_adu_t *adu);
static uint16_t execute_read_write_multiple_registers(mb_adu_t *adu);
static uint16_t execute_mask_write_registers(mb_adu_t *adu);
static uint16_t execute_read_fifo(mb_adu_t *adu);
static uint16_t execute_read_file_record(mb_adu_t *adu);
static uint16_t execute_write_file_record(mb_adu_t *adu);
static uint16_t execute_read_exception_status(mb_adu_t *adu);
static uint16_t execute_diagnostic(mb_adu_t *adu);
static uint16_t execute_get_com_event_counter(mb_adu_t *adu);
static uint16_t execute_get_com_event_log(mb_adu_t *adu);
static uint16_t execute_report_device_id(mb_adu_t *adu);
static uint16_t execute_encapsulate_tp_info(mb_adu_t *adu);
/*STANDARD FUNCTIONS HANDLERS END*/

/*local variables*/

static mb_client_device_t* m_device = NULL;
static mb_counters_t m_counters = {0};
static uint8_t m_exception_status = 0x00; //nothing is happened here.

/*local variables END*/
static inline void clear_counters() {
  m_counters.bus_char_overrrun = 0;
  m_counters.bus_com_err = 0;
  m_counters.bus_msg = 0;
  m_counters.exc_err = 0;
  m_counters.slave_busy = 0;
  m_counters.slave_msg = 0;
  m_counters.slave_NAK = 0;
  m_counters.slave_no_resp = 0;
}
//////////////////////////////////////////////////////////////////////////

void
mb_init(mb_client_device_t *dev) {
  m_device = dev;
  clear_counters();
}
////////////////////////////////////////////////////////////////////////////

void
handle_broadcast_message(uint8_t *data, uint16_t len) {
  UNUSED_ARG(data);
  UNUSED_ARG(len);
  //do something. maybe go to silent mode, I don't know
}

static volatile uint8_t is_busy = 0;
uint16_t
mb_handle_request(uint8_t *data, uint16_t data_len) {
  uint16_t res = 0x00; //success
  mb_adu_t *adu_req = NULL;
  uint8_t *adu_old_data = NULL;
  mb_request_handler_t *rh = NULL;
  uint16_t expected_crc, real_crc;

  do {
    if (is_busy) {
      ++m_counters.slave_busy;
      break; //maybe we need to handle this somehow?
    }

    is_busy = 1;
    if (data_len < 3) {
      ++m_counters.bus_com_err;
      break;
    }

    real_crc = U16_LSBFromStream(data + data_len - 2);
    expected_crc = crc16(data, data_len - 2);

    if (real_crc != expected_crc) {
      ++m_counters.bus_com_err;
      break;
    }

    ++m_counters.bus_msg;

    adu_req = adu_from_stream(data, data_len);
    adu_old_data = adu_req->data;
    rh = mb_validate_function_code(adu_req);

    if (adu_req->addr == 0) {
      handle_broadcast_message(data, data_len);
      ++m_counters.slave_msg;
      ++m_counters.slave_no_resp;
      break;
    }

    if (adu_req->addr != m_device->address)
      break; //silently.

    m_counters.slave_msg++;
    if (!rh->fc_validation_result) {
      ++m_counters.exc_err;
      mb_send_exc_response(res = mbec_illegal_function, adu_req);
      break;
    }

    if (!rh->pf_check_address(adu_req)) {
      ++m_counters.exc_err;
      mb_send_exc_response(res = mbec_illegal_data_address, adu_req);
      break;
    }

    if (!rh->pf_validate_data_value(adu_req)) {
      ++m_counters.exc_err;
      mb_send_exc_response(res = mbec_illegal_data_value, adu_req);
      break;
    }

    if ((res = rh->pf_execute_function(adu_req))) {
      ++m_counters.exc_err;
      mb_send_exc_response(res, adu_req);
      break;
    }

    res = mb_send_response(adu_req);
  } while(0);

  if (adu_req) {
    if (adu_req->data && adu_req->data != adu_old_data)
      hm_free((memory_t)adu_req->data); //allocated in pf_execute_functions
    hm_free((memory_t)adu_req); //allocated in adu_from_stream
  }

  is_busy = 0;
  return res;
}
////////////////////////////////////////////////////////////////////////////

uint16_t
check_read_discrete_input_data(mb_adu_t *adu) {
  uint16_t address = U16_MSBFromStream(adu->data);
  uint16_t quantity = U16_MSBFromStream(adu->data+2);
  uint16_t bl = nearestMultipleOf8(quantity) / 8;

  return (quantity >= 1 && quantity <= 0x07d0) &&
      (bl + address / 8 < m_device->input_discrete_map.end_addr &&
       address / 8 >= m_device->input_discrete_map.start_addr);
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_read_coils_data(mb_adu_t *adu) {
  uint16_t address = U16_MSBFromStream(adu->data);
  uint16_t quantity = U16_MSBFromStream(adu->data+2);
  uint16_t bn = nearestMultipleOf8(quantity) / 8;

  return (quantity >= 1 && quantity <= 0x07d0) &&
      (bn + address / 8 < m_device->coils_map.end_addr &&
       address >= m_device->coils_map.start_addr);
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_write_single_coil_data(mb_adu_t *adu) {
  uint16_t address = U16_MSBFromStream(adu->data);
  uint16_t coil_state = U16_MSBFromStream(adu->data+2);
  if (coil_state != coin_state_off && coil_state != coin_state_on)
    return 0u;

  return address / 8 >= m_device->coils_map.start_addr &&
      address / 8 < m_device->coils_map.end_addr;
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_write_multiple_coils_data(mb_adu_t *adu) {
  uint16_t address = U16_MSBFromStream(adu->data);
  uint16_t quantity = U16_MSBFromStream(adu->data + 2);
  uint8_t byte_count = *(adu->data + 4);

  return (quantity >= 1 && quantity <= 0x07d0) &&
      (byte_count == nearestMultipleOf8(quantity) / 8) &&
      (address / 8 >= m_device->coils_map.start_addr &&
       address / 8 + byte_count < m_device->coils_map.end_addr);
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_read_input_registers_data(mb_adu_t *adu) {
  uint16_t address = U16_MSBFromStream(adu->data);
  uint16_t quantity = U16_MSBFromStream(adu->data+2);

  return (quantity >= 1 && quantity <= 0x007d) &&
      (address >= m_device->input_registers_map.start_addr) &&
      (quantity + address < m_device->input_registers_map.end_addr);
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_read_holding_registers_data(mb_adu_t *adu) {
  uint16_t address = U16_MSBFromStream(adu->data);
  uint16_t quantity = U16_MSBFromStream(adu->data+2);

  return (quantity >= 1 && quantity <= 0x007d) &&
      (address >= m_device->holding_registers_map.start_addr &&
       quantity + address < m_device->holding_registers_map.end_addr);
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_write_single_register_data(mb_adu_t *adu) {
  uint16_t address = U16_MSBFromStream(adu->data);
  return address >= m_device->holding_registers_map.start_addr &&
      address < m_device->holding_registers_map.end_addr;
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_write_multiple_registers_data(mb_adu_t *adu) {
  uint16_t address = U16_MSBFromStream(adu->data);
  uint16_t quantity = U16_MSBFromStream(adu->data + 2);
  uint8_t byte_count = *(adu->data + 4);

  return quantity >= 1 &&
      quantity <= 0x0079 &&
      byte_count == quantity * 2 &&
      address >= m_device->holding_registers_map.start_addr &&
      address + quantity < m_device->holding_registers_map.end_addr;
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_read_write_multiple_registers_data(mb_adu_t *adu) {
  uint16_t read_start_addr = U16_MSBFromStream(adu->data);
  uint16_t read_quantity = U16_MSBFromStream(adu->data + 2);
  uint16_t write_start_addr = U16_MSBFromStream(adu->data + 4);
  uint16_t write_quantity = U16_MSBFromStream(adu->data + 6);
  uint8_t write_byte_count = *(adu->data + 8);

  return read_quantity >= 1 && read_quantity <= 0x007d &&
      write_quantity >= 1 && write_quantity <= 0x0079 &&
      write_byte_count == write_quantity * 2 &&
      read_start_addr + read_quantity < m_device->holding_registers_map.end_addr &&
      write_start_addr + write_quantity < m_device->holding_registers_map.end_addr;
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_mask_write_registers_data(mb_adu_t *adu) {
  uint16_t address = U16_MSBFromStream(adu->data);
  return address >= m_device->holding_registers_map.start_addr &&
      address < m_device->holding_registers_map.end_addr;
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_read_fifo_data(mb_adu_t *adu) {
  UNUSED_ARG(adu);
  return 0u;
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_read_file_record_data(mb_adu_t *adu) {
  UNUSED_ARG(adu);
  return 0u;
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_write_file_record_data(mb_adu_t *adu) {
  UNUSED_ARG(adu);
  return 0u;
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_read_exception_status_data(mb_adu_t *adu) {
  UNUSED_ARG(adu);
  return 1u;
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_diagnostic_data(mb_adu_t *adu) {
  uint16_t sub_function = U16_MSBFromStream(adu->data);
  return sub_function < sizeof(diagnostic_data_handlers) / sizeof(pf_diagnostic_data_t)
      && diagnostic_data_handlers[sub_function];
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_get_com_event_counter_data(mb_adu_t *adu) {
  UNUSED_ARG(adu);
  return 1u;
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_get_com_event_log_data(mb_adu_t *adu) {
  UNUSED_ARG(adu);
  return 0u;
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_report_device_id_data(mb_adu_t *adu) {
  UNUSED_ARG(adu);
  return 1u; //always return 1 because there is no data in request
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_encapsulate_tp_info_data(mb_adu_t *adu) {
  uint8_t mei_type = *(adu->data);
  return mei_type == 0x0d || mei_type == 0x0e;
}
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

/*execute functions*/

uint16_t mb_read_bits(mb_adu_t *adu, uint8_t *real_addr) {
  register uint16_t bc, rbn;
  register uint8_t i;
  register uint8_t rshift;
  register uint8_t* tmp ;
  uint16_t address = U16_MSBFromStream(adu->data);
  uint16_t quantity = U16_MSBFromStream(adu->data+2);

  bc = nearestMultipleOf8(quantity) / 8;
  adu->data_len = bc + 1;

  if (!(adu->data = (uint8_t*) hm_malloc(adu->data_len + 1)))
    return mbec_heap_error;

  adu->data[0] = bc;
  tmp = adu->data + 1;

  rshift = address % 8;
  rbn = address / 8;
  while (bc--) {
    for (i = 0; i < 8; ++i) {
      *tmp >>= 1;
      if (real_addr[rbn] & (0x80 >> rshift))
        *tmp |= 0x80;
      else
        *tmp &= ~0x80;

      if (++rshift != 8) continue;
      ++rbn;
      rshift = 0;
    }
    ++tmp;
  }

  return mbec_OK;
}

uint16_t execute_read_discrete_inputs(mb_adu_t *adu) {
  return mb_read_bits(adu, m_device->input_discrete_map.real_addr);
}
//////////////////////////////////////////////////////////////////////////

uint16_t execute_read_coils(mb_adu_t *adu) {
  return mb_read_bits(adu, m_device->coils_map.real_addr);
}
//////////////////////////////////////////////////////////////////////////

uint16_t execute_write_single_coil(mb_adu_t *adu) {
  uint16_t address = U16_MSBFromStream(adu->data);
  uint16_t coil_state = U16_MSBFromStream(adu->data+2);
  if (coil_state == coin_state_off)
    m_device->coils_map.real_addr[address / 8] &= ~(0x80 >> address % 8);
  else
    m_device->coils_map.real_addr[address / 8] |= (0x80 >> address % 8);
  //we don't do anything with adu, should return it as is
  return mbec_OK ;
}
//////////////////////////////////////////////////////////////////////////

uint16_t execute_write_multiple_coils(mb_adu_t *adu) {
  register int8_t i;
  register uint16_t ba;
  register uint8_t shift;

  uint16_t address = U16_MSBFromStream(adu->data);
  uint16_t quantity = U16_MSBFromStream(adu->data + 2);
  uint8_t byte_count = *(adu->data + 4);
  uint8_t *data = adu->data + 5;

  ba = address / 8;
  shift = address % 8;

  while (byte_count--) {
    for (i = 0; i < 8 && quantity--; ++i) {
      if (*data & 0x01)
        m_device->coils_map.real_addr[ba] |= (0x80 >> shift);
      else
        m_device->coils_map.real_addr[ba] &= ~(0x80 >> shift);
      *data >>= 1;

      if (++shift != 8) continue;
      shift = 0;
      ++ba;
    } //for
    ++data;
  } //while

  //we don't do anything with adu, should return it as is
  return mbec_OK;
}
//////////////////////////////////////////////////////////////////////////

uint16_t mb_read_registers(mb_adu_t *adu,
                           uint16_t *real_addr) {
  int16_t i;
  uint8_t* tmp;
  uint16_t address = U16_MSBFromStream(adu->data);
  uint16_t quantity = U16_MSBFromStream(adu->data + 2);
  adu->data_len = quantity*sizeof(mb_register) + 1;
  adu->data = (uint8_t*) hm_malloc(adu->data_len);
  if (!adu->data)
    return mbec_heap_error;

  adu->data[0] = adu->data_len - 1;
  tmp = adu->data + 1;

  for (i = 0; i < quantity; ++i, tmp += sizeof(mb_register)) {
    U16_MSB2Stream(real_addr[address + i], tmp);
  }
  return mbec_OK;
}

uint16_t execute_read_input_registers(mb_adu_t *adu) {
  return mb_read_registers(adu, m_device->input_registers_map.real_addr);
}
//////////////////////////////////////////////////////////////////////////

uint16_t execute_read_holding_registers(mb_adu_t *adu) {
  return mb_read_registers(adu, m_device->holding_registers_map.real_addr);
}
//////////////////////////////////////////////////////////////////////////

uint16_t execute_write_single_register(mb_adu_t *adu) {
  uint16_t address = U16_MSBFromStream(adu->data);
  uint16_t data = U16_MSBFromStream(adu->data+2);
  m_device->holding_registers_map.real_addr[address] = data;
  //we don't do anything with adu, should return it as is
  return mbec_OK;
}
//////////////////////////////////////////////////////////////////////////

uint16_t execute_write_multiple_registers(mb_adu_t *adu) {

  uint16_t address = U16_MSBFromStream(adu->data);
  uint16_t quantity = U16_MSBFromStream(adu->data + 2);
  uint8_t byte_count = *(adu->data + 4);
  uint8_t *data = adu->data + 5;

  uint8_t* tmp;
  if (!(adu->data = (uint8_t*) hm_malloc(4)))
    return mbec_heap_error;
  adu->data_len = 4;

  tmp = (uint8_t*) &m_device->holding_registers_map.real_addr[address];
  while (byte_count--)
    *(tmp++) = *(data++);

  U16_MSB2Stream(address, adu->data);
  U16_MSB2Stream(quantity, adu->data+2);
  return mbec_OK;
}
//////////////////////////////////////////////////////////////////////////

uint16_t execute_read_write_multiple_registers(mb_adu_t *adu) {
  uint16_t read_start_addr = U16_MSBFromStream(adu->data);
  uint16_t read_quantity = U16_MSBFromStream(adu->data + 2);
  uint16_t write_start_addr = U16_MSBFromStream(adu->data + 4);
  uint8_t write_byte_count = *(adu->data + 8);
  uint8_t *write_data = adu->data + 9;
  uint8_t* tmp;
  uint16_t i;

  adu->data_len = read_quantity*sizeof(mb_register) + 1;
  adu->data = (uint8_t*) hm_malloc(adu->data_len);
  if (!adu->data)
    return mbec_heap_error;

  adu->data[0] = adu->data_len - 1;
  tmp = adu->data + 1;
  for (i = 0; i < read_quantity; ++i, tmp += sizeof(mb_register)) {
    U16_MSB2Stream(m_device->holding_registers_map.real_addr[read_start_addr + i], tmp);
  }

  tmp = (uint8_t*) &m_device->holding_registers_map.real_addr[write_start_addr];
  while (write_byte_count--)
    *(tmp++) = *(write_data++);

  return mbec_OK;
}
//////////////////////////////////////////////////////////////////////////

//Result = (Current Contents AND And_Mask) OR (Or_Mask AND (NOT And_Mask))
uint16_t execute_mask_write_registers(mb_adu_t *adu) {
  uint16_t address = U16_MSBFromStream(adu->data);
  uint16_t and_mask = U16_MSBFromStream(adu->data+2);
  uint16_t or_mask = U16_MSBFromStream(adu->data+4);

  m_device->holding_registers_map.real_addr[address] =
      (m_device->holding_registers_map.real_addr[address] & and_mask) |
      (or_mask & ~and_mask);
  //we don't do anything with adu, should return it as is
  return mbec_OK;
}
//////////////////////////////////////////////////////////////////////////

uint16_t execute_read_fifo(mb_adu_t *adu) {
  UNUSED_ARG(adu);
  return 0u;
}
//////////////////////////////////////////////////////////////////////////

uint16_t execute_read_file_record(mb_adu_t *adu) {
  UNUSED_ARG(adu);
  return 0u;
}
//////////////////////////////////////////////////////////////////////////

uint16_t execute_write_file_record(mb_adu_t *adu) {
  UNUSED_ARG(adu);
  return 0u;
}
//////////////////////////////////////////////////////////////////////////

uint16_t execute_read_exception_status(mb_adu_t *adu) {
  adu->data_len = 1; //exception status
  adu->data = (uint8_t*) hm_malloc(adu->data_len);
  if (!adu->data)
    return mbec_heap_error;
  *adu->data = m_exception_status;
  return mbec_OK;
}
//////////////////////////////////////////////////////////////////////////

uint16_t diag_return_query_data(mb_adu_t *adu) {
  UNUSED_ARG(adu); //just return adu as is . is it kind of ping?
  return mbec_OK;
}
////////////////////////////////////////////////////////////////////////////

uint16_t diag_restart_communications_option(mb_adu_t *adu) {
  uint16_t clear_communication_event_log = U16_MSBFromStream(adu->data+2);
  switch (clear_communication_event_log) {
    case 0xff00:
      //todo clear_communication_event_log
      break;
    case 0x0000:
      break;
    default:
      return mbec_illegal_data_value;
  }

  //todo restart communications
  clear_counters();
  return mbec_OK;
}
////////////////////////////////////////////////////////////////////////////

uint16_t diag_return_diagnostic_register(mb_adu_t *adu) {
  UNUSED_ARG(adu);
  return mbec_illegal_function;
}
////////////////////////////////////////////////////////////////////////////

uint16_t diag_change_adcii_input_delimiter(mb_adu_t *adu) {
  UNUSED_ARG(adu);
  return mbec_illegal_function;
}
////////////////////////////////////////////////////////////////////////////

uint16_t diag_force_listen_only_mode(mb_adu_t *adu) {
  UNUSED_ARG(adu);
  return mbec_illegal_function;
}
////////////////////////////////////////////////////////////////////////////

uint16_t diag_clean_counter_and_diagnostic_registers(mb_adu_t *adu) {
  UNUSED_ARG(adu);
  clear_counters();
  return mbec_OK;
}
////////////////////////////////////////////////////////////////////////////

static inline uint16_t diag_return_some_counter(mb_adu_t *adu, uint16_t val) {
  uint16_t sf = U16_MSBFromStream(adu->data); //sub function
  if (!(adu->data = (uint8_t*) hm_malloc(4)))
    return mbec_heap_error;
  adu->data_len = 4;
  U16_MSB2Stream(sf, adu->data);
  U16_MSB2Stream(val, adu->data+2);
  return mbec_OK;
}

uint16_t diag_return_bus_messages_count(mb_adu_t *adu) {
  return diag_return_some_counter(adu, m_counters.bus_msg);
}

uint16_t diag_return_bus_communication_error_count(mb_adu_t *adu) {
  return diag_return_some_counter(adu, m_counters.bus_com_err);
}

uint16_t diag_return_bus_exception_error_count(mb_adu_t *adu) {
  return diag_return_some_counter(adu, m_counters.exc_err);
}

uint16_t diag_return_server_messages_count(mb_adu_t *adu) {
  return diag_return_some_counter(adu, m_counters.slave_msg);
}

uint16_t diag_return_server_no_response_count(mb_adu_t *adu) {
  return diag_return_some_counter(adu, m_counters.slave_no_resp);
}

uint16_t diag_return_server_NAK_count(mb_adu_t *adu) {
  return diag_return_some_counter(adu, m_counters.slave_NAK);
}

uint16_t diag_return_server_busy_count(mb_adu_t *adu) {
  return diag_return_some_counter(adu, m_counters.slave_busy);
}

uint16_t diag_return_bus_character_overrun_count(mb_adu_t *adu) {
  return diag_return_some_counter(adu, m_counters.bus_char_overrrun);
}

uint16_t diag_clear_overrun_counter_and_flag(mb_adu_t *adu) {
  UNUSED_ARG(adu);
  m_counters.bus_char_overrrun = 0;
  return mbec_OK;
}

uint16_t execute_diagnostic(mb_adu_t *adu) {
  uint16_t sub_function = U16_MSBFromStream(adu->data);
  pf_diagnostic_data_t handler = diagnostic_data_handlers[sub_function];
  return handler(adu);
}
//////////////////////////////////////////////////////////////////////////

uint16_t execute_get_com_event_counter(mb_adu_t *adu) {  
  UNUSED_ARG(adu); //todo implement this later
  return mbec_illegal_function;
}
//////////////////////////////////////////////////////////////////////////

uint16_t execute_get_com_event_log(mb_adu_t *adu) {
  UNUSED_ARG(adu);
  return mbec_illegal_function;
}
//////////////////////////////////////////////////////////////////////////

uint16_t execute_report_device_id(mb_adu_t *adu) {
  adu->data_len = 2;
  adu->data = (uint8_t*) hm_malloc(adu->data_len);
  if (!adu->data)
    return mbec_heap_error;

  adu->data[0] = m_device->address; //should be some device specific data. now - nothing.
  adu->data[1] = 0xff; //0x00 -OFF, 0xff - ON. Run indicator status
  return mbec_OK;
}
//////////////////////////////////////////////////////////////////////////

uint16_t execute_encapsulate_tp_info(mb_adu_t *adu) {  
  UNUSED_ARG(adu);
  return mbec_illegal_function;
}
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

mb_request_handler_t*
mb_validate_function_code(mb_adu_t* adu) {
  //maybe it's better to use switch, because this table takes ~ 19*(1+2+3*sizeof(funciton_pointer))B
  enum {fc_is_not_supported = 0, fc_is_supported = 1};

  static mb_request_handler_t handlers[] = {
    {mbfc_read_discrete_input, fc_is_supported, check_discrete_input_address,
     check_read_discrete_input_data, execute_read_discrete_inputs },

    {mbfc_read_coils, fc_is_supported, check_coils_address,
     check_read_coils_data, execute_read_coils },

    {mbfc_write_single_coil, fc_is_supported, check_coils_address,
     check_write_single_coil_data, execute_write_single_coil },

    {mbfc_write_multiple_coils, fc_is_supported, check_coils_address,
     check_write_multiple_coils_data, execute_write_multiple_coils },
    /*rw registers*/

    {mbfc_read_input_registers, fc_is_supported, check_input_registers_address,
     check_read_input_registers_data, execute_read_input_registers },

    {mbfc_read_holding_registers, fc_is_supported, check_holding_registers_address,
     check_read_holding_registers_data, execute_read_holding_registers },

    {mbfc_write_single_register, fc_is_supported, check_holding_registers_address,
     check_write_single_register_data, execute_write_single_register },

    {mbfc_write_multiple_registers, fc_is_supported, check_holding_registers_address,
     check_write_multiple_registers_data, execute_write_multiple_registers },

    {mbfc_read_write_multiple_registers, fc_is_not_supported, check_holding_registers_address,
     check_read_write_multiple_registers_data, execute_read_write_multiple_registers },

    {mbfc_mask_write_registers, fc_is_supported, check_holding_registers_address,
     check_mask_write_registers_data, execute_mask_write_registers },

    /*r fifo*/
    {mbfc_read_fifo, fc_is_not_supported, check_address_and_return_ok,
     check_read_fifo_data, execute_read_fifo },
    /*diagnostic*/

    {mbfc_read_file_record, fc_is_not_supported, check_address_and_return_ok,
     check_read_file_record_data, execute_read_file_record },

    {mbfc_write_file_record, fc_is_not_supported, check_address_and_return_ok,
     check_write_file_record_data, execute_write_file_record },

    {mbfc_read_exception_status, fc_is_not_supported, check_address_and_return_ok,
     check_read_exception_status_data, execute_read_exception_status },

    {mbfc_diagnostic, fc_is_supported, check_address_and_return_ok,
     check_diagnostic_data, execute_diagnostic },

    {mbfc_get_com_event_counter, fc_is_not_supported, check_address_and_return_ok,
     check_get_com_event_counter_data, execute_get_com_event_counter },

    {mbfc_get_com_event_log, fc_is_supported, check_address_and_return_ok,
     check_get_com_event_log_data, execute_get_com_event_log },

    /*misc*/
    {mbfc_report_device_id, fc_is_supported, check_address_and_return_ok,
     check_report_device_id_data, execute_report_device_id },

    //strange function. we will support only one parameter : 0x0e
    {mbfc_encapsulate_tp_info, fc_is_supported, check_address_and_return_ok,
     check_encapsulate_tp_info_data, execute_encapsulate_tp_info },

    {0xff, fc_is_not_supported, NULL, NULL, NULL} /*UNSUPPORTED FUNCTION HANDLER*/
  }; //handlers table

  mb_request_handler_t* res = handlers;
  for (; res->fc != 0xff; ++res) {
    if (res->fc == adu->fc) break;
  }

  return res;
}
////////////////////////////////////////////////////////////////////////////

uint16_t
mb_send_response(mb_adu_t* adu) {
  uint8_t* send_buff = adu_serialize(adu);
  if (!send_buff) return mbec_heap_error;
  m_device->tp_send(send_buff, adu_buffer_len(adu));
  hm_free((memory_t)send_buff);
  return 0u;
}
////////////////////////////////////////////////////////////////////////////

void
mb_send_exc_response(mbec_exception_code_t exc_code, mb_adu_t* adu) {
  uint8_t resp[5] = {adu->addr,
                     adu->fc | 0x80,
                     exc_code };
  U16_LSB2Stream(crc16(resp, 3), resp + 3);
  m_device->tp_send(resp, 5);
}
////////////////////////////////////////////////////////////////////////////

mb_adu_t*
adu_from_stream(uint8_t *data, uint16_t len) {
  mb_adu_t* result = (mb_adu_t*) hm_malloc(sizeof(mb_adu_t));
  if (!result) return result;

  result->addr = *(uint8_t*)data;
  data += sizeof(result->addr);
  result->fc = *data;
  data += sizeof(result->fc);
  result->data = data;
  result->data_len = len - (sizeof(mb_adu_t) -
                            sizeof(result->data) -
                            sizeof(result->data_len));
  data += result->data_len;
  result->crc = U16_LSBFromStream(data);
  return result;
}
////////////////////////////////////////////////////////////////////////////

uint8_t*
adu_serialize(mb_adu_t *adu) {
  uint16_t i, crc;
  uint8_t *tmp;
  uint8_t *buffer = (uint8_t*) hm_malloc(adu_buffer_len(adu));
  if (!buffer) return NULL;

  tmp = buffer;
  *tmp = adu->addr;
  tmp += sizeof(adu->addr);
  *tmp = adu->fc;
  tmp += sizeof(adu->fc);

  for (i = 0; i < adu->data_len; ++i, ++tmp) {
    *tmp = adu->data[i];
  }

  crc = crc16(buffer, adu_buffer_len(adu) - sizeof(crc));
  U16_LSB2Stream(crc, tmp);
  return buffer;
}
//////////////////////////////////////////////////////////////////////////

static inline uint8_t
valid_register_addr(mb_dev_registers_mapping_t *mapping, uint16_t addr) {
  return (addr >= mapping->start_addr && addr < mapping->end_addr);
}

static inline uint8_t
valid_bit_addr(mb_dev_bit_mapping_t *mapping, uint16_t bit_addr) {
  uint16_t n8 = nearestMultipleOf8(bit_addr) / 8;
  return n8 >= mapping->start_addr && n8 < mapping->end_addr;
}

uint16_t
check_discrete_input_address(mb_adu_t *adu) {
  uint16_t addr = U16_MSBFromStream(adu->data);
  return m_device && valid_bit_addr(&m_device->input_discrete_map, addr);
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_coils_address(mb_adu_t *adu) {
  uint16_t addr = U16_MSBFromStream(adu->data);
  return m_device && valid_bit_addr(&m_device->coils_map, addr);
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_input_registers_address(mb_adu_t *adu) {
  uint16_t addr = U16_MSBFromStream(adu->data);
  return m_device && valid_register_addr(&m_device->input_registers_map, addr);
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_holding_registers_address(mb_adu_t *adu) {
  uint16_t addr = U16_MSBFromStream(adu->data);
  return m_device && valid_register_addr(&m_device->holding_registers_map, addr);
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_address_and_return_ok() {
  return 1u;
}
//////////////////////////////////////////////////////////////////////////
