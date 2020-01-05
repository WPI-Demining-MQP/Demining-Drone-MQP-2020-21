#include "BaseStationComms.h"

HardwareSerial* comms_port = &Serial2;  // pointer to the Serial port to be used by the base station comms - defaults to Serial2

void setup_comms(HardwareSerial* serial_ptr, uint32_t comms_baud) {
  comms_port = serial_ptr;
  comms_port->begin(comms_baud);
}

void send_message(uint8_t msg_type, uint8_t data_len, uint8_t* data) {
  uint8_t parity = generate_parity(data, data_len);
  comms_port->write(START_BYTE);
  comms_port->write(msg_type);
  comms_port->write(data_len);
  if(data_len != 0)
    comms_port->write(data, data_len);
  comms_port->write(parity);
}

// XOR's all data bytes together to get a parity byte
uint8_t generate_parity(uint8_t *data, uint8_t len) {
  uint8_t checksum = 0;
  for(int i = 0; i < len; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

// Sends a heartbeat message to the base station
void send_msg_heartbeat() {
  send_message(MSG_HEARTBEAT, 0, NULL);
}

// Sends a status message containing the passed string
void send_msg_status(const char* msg) {
  uint8_t data_len = 0;
  while(msg[data_len] != '\0') {
    data_len++;
  }
  send_message(MSG_STATUS, data_len, (uint8_t*)msg);
}

// parses a received minefield message and returns the total number of mines that are present
uint16_t parse_msg_minefield(uint8_t* data) {
  return(*(uint16_t*)data);
}

// parses the lat/lon of a single mine
void parse_msg_mine(uint8_t* data, uint32_t* lat_ptr, uint32_t* lon_ptr) {
  *lat_ptr = *((uint32_t*)data);
  *lon_ptr = *((uint32_t*)(data + 4));
}
