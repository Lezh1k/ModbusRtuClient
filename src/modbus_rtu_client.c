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
  uint16_t bus_char_over; //cpt8 return bus character overrun count
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

static void handle_incomplete_message(uint8_t *data, uint16_t len);
static void handle_wrong_crc(uint16_t expected_crc, uint16_t real_crc);
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

typedef uint8_t* (*pf_diagnostic_data_t)(mb_adu_t *adu);
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

void
mb_init(mb_client_device_t *dev) {
  m_device = dev;
}
////////////////////////////////////////////////////////////////////////////

void
handle_broadcast_message(uint8_t *data, uint16_t len) {
  UNUSED_ARG(data);
  UNUSED_ARG(len);
#pragma message ("TODO  implement handle_broadcast_message")
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
      m_counters.slave_busy++;
      break; //maybe we need to handle this somehow?
    }

    is_busy = 1;
    if (data_len < 3) {
      m_counters.bus_com_err++;
      break;
    }

    real_crc = U16_LSB_from_stream(data + data_len - 2);
    expected_crc = crc16(data, data_len - 2);

    if (real_crc != expected_crc) {
      m_counters.bus_com_err++;
      break;
    }

    m_counters.bus_msg++;

    adu_req = adu_from_stream(data, data_len);
    adu_old_data = adu_req->data;
    rh = mb_validate_function_code(adu_req);

    if (adu_req->addr == 0) {
      handle_broadcast_message(data, data_len);
      m_counters.slave_msg++;
      m_counters.slave_no_resp++;
      break;
    }

    if (adu_req->addr != m_device->address)
      break; //silently.

    m_counters.slave_msg++;
    if (!rh->fc_validation_result) {
      m_counters.exc_err++;
      mb_send_exc_response(res = mbec_illegal_function, adu_req);
      break;
    }

    if (!rh->pf_check_address(adu_req)) {
      m_counters.exc_err++;
      mb_send_exc_response(res = mbec_illegal_data_address, adu_req);
      break;
    }

    if (!rh->pf_validate_data_value(adu_req)) {
      m_counters.exc_err++;
      mb_send_exc_response(res = mbec_illegal_data_value, adu_req);
      break;
    }

    if ((res = rh->pf_execute_function(adu_req))) {
      m_counters.exc_err++;
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
  read_bits_arg_t arg = {
    .address = U16_MSB_from_stream(adu->data),
    .quantity = U16_MSB_from_stream(adu->data+2)
  };
  uint16_t bl = nearest_8_multiple(arg.quantity) / 8;

  return (arg.quantity >= 1 && arg.quantity <= 0x07d0) &&
      (bl + arg.address / 8 < m_device->input_discrete_map.end_addr &&
       arg.address / 8 >= m_device->input_discrete_map.start_addr);
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_read_coils_data(mb_adu_t *adu) {
  read_bits_arg_t arg = {
    .address = U16_MSB_from_stream(adu->data),
    .quantity = U16_MSB_from_stream(adu->data+2)
  };
  uint16_t bn = nearest_8_multiple(arg.quantity) / 8;

  return (arg.quantity >= 1 && arg.quantity <= 0x07d0) &&
      (bn + arg.address / 8 < m_device->coils_map.end_addr &&
       arg.address >= m_device->coils_map.start_addr);
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_write_single_coil_data(mb_adu_t *adu) {
  write_single_coil_arg_t arg = {
    .address = U16_MSB_from_stream(adu->data),
    .coil_state = U16_MSB_from_stream(adu->data+2)
  };

  if (arg.coil_state != cs_off && arg.coil_state != cs_on)
    return 0u;

  return arg.address / 8 >= m_device->coils_map.start_addr &&
      arg.address / 8 < m_device->coils_map.end_addr;
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_write_multiple_coils_data(mb_adu_t *adu) {
  write_multiple_coils_arg_t arg = {
    .address = U16_MSB_from_stream(adu->data),
    .quantity = U16_MSB_from_stream(adu->data + 2),
    .byte_count = *(adu->data + 4),
    .data = adu->data + 5
  };

  return (arg.quantity >= 1 && arg.quantity <= 0x07d0) &&
      (arg.byte_count == nearest_8_multiple(arg.quantity) / 8) &&
      (arg.address / 8 >= m_device->coils_map.start_addr &&
       arg.address / 8 + arg.byte_count < m_device->coils_map.end_addr);
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_read_input_registers_data(mb_adu_t *adu) {
  read_registers_arg_t arg = {
    .address = U16_MSB_from_stream(adu->data),
    .quantity = U16_MSB_from_stream(adu->data+2)
  };

  return (arg.quantity >= 1 && arg.quantity <= 0x007d) &&
      (arg.address >= m_device->input_registers_map.start_addr) &&
      (arg.quantity + arg.address < m_device->input_registers_map.end_addr);
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_read_holding_registers_data(mb_adu_t *adu) {
  read_registers_arg_t arg = {
    .address = U16_MSB_from_stream(adu->data),
    .quantity = U16_MSB_from_stream(adu->data+2)
  };

  return (arg.quantity >= 1 && arg.quantity <= 0x007d) &&
      (arg.address >= m_device->holding_registers_map.start_addr &&
       arg.quantity + arg.address < m_device->holding_registers_map.end_addr);
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_write_single_register_data(mb_adu_t *adu) {
  write_single_register_arg_t arg = {
    .address = U16_MSB_from_stream(adu->data),
    .data = U16_MSB_from_stream(adu->data+2)
  };

  return arg.address >= m_device->holding_registers_map.start_addr &&
      arg.address < m_device->holding_registers_map.end_addr;
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_write_multiple_registers_data(mb_adu_t *adu) {
  write_multiple_registers_arg_t arg = {
    .address = U16_MSB_from_stream(adu->data),
    .quantity = U16_MSB_from_stream(adu->data + 2),
    .byte_count = *(adu->data + 5),
    .data = adu->data + 6
  };

  return arg.address >= m_device->holding_registers_map.start_addr &&
      arg.address + arg.quantity < m_device->holding_registers_map.end_addr;
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_read_write_multiple_registers_data(mb_adu_t *adu) {
  UNUSED_ARG(adu);
  return 0;
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_mask_write_registers_data(mb_adu_t *adu) {
  mask_write_register_arg_t arg = {
    .address = U16_MSB_from_stream(adu->data),
    .and_mask = U16_MSB_from_stream(adu->data+2),
    .or_mask = U16_MSB_from_stream(adu->data+4)
  };

  return arg.address >= m_device->holding_registers_map.start_addr &&
      arg.address < m_device->holding_registers_map.end_addr;
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
  uint16_t sub_function = U16_MSB_from_stream(adu->data);
  return sub_function < sizeof(diagnostic_data_handlers) / sizeof(pf_diagnostic_data_t)
      && diagnostic_data_handlers[sub_function];
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_get_com_event_counter_data(mb_adu_t *adu) {
  UNUSED_ARG(adu);
  return 0u;
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
  UNUSED_ARG(adu);
  return 0u;
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
  read_bits_arg_t arg = {
    .address = U16_MSB_from_stream(adu->data),
    .quantity = U16_MSB_from_stream(adu->data+2)
  };

  bc = nearest_8_multiple(arg.quantity) / 8;
  adu->data_len = bc + 1;

  if (!(adu->data = (uint8_t*) hm_malloc(adu->data_len + 1)))
    return mbec_heap_error;

  adu->data[0] = bc;
  tmp = adu->data + 1;

  rshift = arg.address % 8;
  rbn = arg.address / 8;
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
  write_single_coil_arg_t arg = {
    .address = U16_MSB_from_stream(adu->data),
    .coil_state = U16_MSB_from_stream(adu->data+2)
  };

  if (arg.coil_state == cs_off)
    m_device->coils_map.real_addr[arg.address / 8] &= ~(0x80 >> arg.address % 8);
  else
    m_device->coils_map.real_addr[arg.address / 8] |= (0x80 >> arg.address % 8);
  //we don't do anything with adu, should return it as is
  return mbec_OK ;
}
//////////////////////////////////////////////////////////////////////////

uint16_t execute_write_multiple_coils(mb_adu_t *adu) {
  register int8_t i;
  register uint16_t ba;
  register uint8_t shift;

  write_multiple_coils_arg_t arg = {
    .address = U16_MSB_from_stream(adu->data),
    .quantity = U16_MSB_from_stream(adu->data + 2),
    .byte_count = *(adu->data + 4),
    .data = adu->data + 5
  };

  ba = arg.address / 8;
  shift = arg.address % 8;

  while (arg.byte_count--) {
    for (i = 0; i < 8; ++i) {
      if (*arg.data & 0x01)
        m_device->coils_map.real_addr[ba] |= (0x80 >> shift);
      else
        m_device->coils_map.real_addr[ba] &= ~(0x80 >> shift);
      *arg.data >>= 1;

      if (++shift != 8) continue;
      shift = 0;
      ++ba;
    } //for
    ++arg.data;
  } //while

  //we don't do anything with adu, should return it as is
  return mbec_OK;
}
//////////////////////////////////////////////////////////////////////////

uint16_t mb_read_registers(mb_adu_t *adu,
                           uint16_t *real_addr) {
  int16_t i;
  uint8_t* tmp;
  read_registers_arg_t arg = {
    .address = U16_MSB_from_stream(adu->data),
    .quantity = U16_MSB_from_stream(adu->data + 2)
  };

  adu->data_len = arg.quantity*sizeof(mb_register) + 1;
  adu->data = (uint8_t*) hm_malloc(adu->data_len);
  if (!adu->data)
    return mbec_heap_error;

  adu->data[0] = adu->data_len - 1;
  tmp = adu->data + 1;

  for (i = 0; i < arg.quantity; ++i, tmp += sizeof(mb_register)) {
    U16_MSB_to_stream(real_addr[arg.address + i], tmp);
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
  write_single_register_arg_t arg = {
    .address = U16_MSB_from_stream(adu->data),
    .data = U16_MSB_from_stream(adu->data+2)
  };
  m_device->holding_registers_map.real_addr[arg.address] = arg.data;
  //we don't do anything with adu, should return it as is
  return mbec_OK;
}
//////////////////////////////////////////////////////////////////////////

uint16_t execute_write_multiple_registers(mb_adu_t *adu) {
  write_multiple_registers_arg_t arg = {
    .address = U16_MSB_from_stream(adu->data),
    .quantity = U16_MSB_from_stream(adu->data + 2),
    .byte_count = *(adu->data + 4),
    .data = adu->data + 5
  };
  uint8_t* tmp;
  if (!(adu->data = (uint8_t*) hm_malloc(4)))
    return mbec_heap_error;
  adu->data_len = 4;

  tmp = (uint8_t*) &m_device->holding_registers_map.real_addr[arg.address];
  while (arg.byte_count--)
    *(tmp++) = *(arg.data++);

  U16_MSB_to_stream(arg.address, adu->data);
  U16_MSB_to_stream(arg.quantity, adu->data+2);
  return mbec_OK;
}
//////////////////////////////////////////////////////////////////////////

uint16_t execute_read_write_multiple_registers(mb_adu_t *adu) {
  UNUSED_ARG(adu);
  return 0u;
}
//////////////////////////////////////////////////////////////////////////

//Result = (Current Contents AND And_Mask) OR (Or_Mask AND (NOT And_Mask))
uint16_t execute_mask_write_registers(mb_adu_t *adu) {
  mask_write_register_arg_t arg = {
    .address = U16_MSB_from_stream(adu->data),
    .and_mask = U16_MSB_from_stream(adu->data+2),
    .or_mask = U16_MSB_from_stream(adu->data+4)
  };

  m_device->holding_registers_map.real_addr[arg.address] =
      (m_device->holding_registers_map.real_addr[arg.address] & arg.and_mask) |
      (arg.or_mask & ~arg.and_mask);
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
  return adu;
}
//////////////////////////////////////////////////////////////////////////

uint16_t diag_return_query_data(mb_adu_t *adu) {
  UNUSED_ARG(adu); //just return adu as is . is it kind of ping?
  return mbec_OK;
}

uint16_t diag_restart_communications_option(mb_adu_t *adu) {
  return mbec_OK;
}

uint16_t diag_return_diagnostic_register(mb_adu_t *adu) {
  return mbec_OK;
}

uint16_t diag_change_adcii_input_delimiter(mb_adu_t *adu) {
  return mbec_OK;
}

uint16_t diag_force_listen_only_mode(mb_adu_t *adu) {
  return mbec_OK;
}

uint16_t diag_clean_counter_and_diagnostic_registers(mb_adu_t *adu) {
  return mbec_OK;
}

uint16_t diag_return_bus_messages_count(mb_adu_t *adu) {
  return mbec_OK;
}

uint16_t diag_return_bus_communication_error_count(mb_adu_t *adu) {
  return mbec_OK;
}

uint16_t diag_return_bus_exception_error_count(mb_adu_t *adu) {
  return mbec_OK;
}

uint16_t diag_return_server_messages_count(mb_adu_t *adu) {
  return mbec_OK;
}

uint16_t diag_return_server_no_response_count(mb_adu_t *adu) {
  return mbec_OK;
}

uint16_t diag_return_server_NAK_count(mb_adu_t *adu) {
  return mbec_OK;
}

uint16_t diag_return_server_busy_count(mb_adu_t *adu) {
  return mbec_OK;
}

uint16_t diag_return_bus_character_overrun_count(mb_adu_t *adu) {
  return mbec_OK;
}

uint16_t diag_clear_overrun_counter_and_flag(mb_adu_t *adu) {
  return mbec_OK;
}

uint16_t execute_diagnostic(mb_adu_t *adu) {
  uint16_t sub_function = U16_MSB_from_stream(adu->data);
  pf_diagnostic_data_t handler = diagnostic_data_handlers[sub_function];
  return handler(adu);
}
//////////////////////////////////////////////////////////////////////////

uint16_t execute_get_com_event_counter(mb_adu_t *adu) {
  return mbec_OK;
}
//////////////////////////////////////////////////////////////////////////

uint16_t execute_get_com_event_log(mb_adu_t *adu) {
  return mbec_OK;
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
  return mbec_OK;
}
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

mb_request_handler_t*
mb_validate_function_code(mb_adu_t* adu) {
  //maybe it's better to use switch
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

    {mbfc_get_com_event_counter, fc_is_supported, check_address_and_return_ok,
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
  U16_LSB_to_stream(crc16(resp, 3), resp + 3);
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
  result->crc = U16_LSB_from_stream(data);
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
  U16_LSB_to_stream(crc, tmp);
  return buffer;
}
//////////////////////////////////////////////////////////////////////////

static inline uint8_t
valid_register_addr(mb_dev_registers_mapping_t *mapping, uint16_t addr) {
  return (addr >= mapping->start_addr && addr < mapping->end_addr);
}

static inline uint8_t
valid_bit_addr(mb_dev_bit_mapping_t *mapping, uint16_t bit_addr) {
  uint16_t n8 = nearest_8_multiple(bit_addr) / 8;
  return n8 >= mapping->start_addr && n8 < mapping->end_addr;
}

uint16_t
check_discrete_input_address(mb_adu_t *adu) {
  uint16_t addr = U16_MSB_from_stream(adu->data);
  return m_device && valid_bit_addr(&m_device->input_discrete_map, addr);
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_coils_address(mb_adu_t *adu) {
  uint16_t addr = U16_MSB_from_stream(adu->data);
  return m_device && valid_bit_addr(&m_device->coils_map, addr);
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_input_registers_address(mb_adu_t *adu) {
  uint16_t addr = U16_MSB_from_stream(adu->data);
  return m_device && valid_register_addr(&m_device->input_registers_map, addr);
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_holding_registers_address(mb_adu_t *adu) {
  uint16_t addr = U16_MSB_from_stream(adu->data);
  return m_device && valid_register_addr(&m_device->holding_registers_map, addr);
}
//////////////////////////////////////////////////////////////////////////

uint16_t
check_address_and_return_ok() {
  return 1u;
}
//////////////////////////////////////////////////////////////////////////