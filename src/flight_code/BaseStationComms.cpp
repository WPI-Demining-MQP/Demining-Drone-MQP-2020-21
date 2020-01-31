#include "BaseStationComms.h"

HardwareSerial* comms_port = &Serial2;  // pointer to the Serial port to be used by the base station comms - defaults to Serial2
enum system_state_t system_state = SYS_STATE_UNINIT;       // Global system state (reported to base station via heartbeat messages)

void setup_comms(HardwareSerial* serial_ptr, uint32_t comms_baud) {
  comms_port = serial_ptr;
  comms_port->begin(comms_baud);
}

void send_message(uint8_t msg_type, uint8_t data_len, uint8_t* data) {
  uint8_t parity = generate_parity(data, data_len);
  parity = accumulate_parity(parity, START_BYTE);
  parity = accumulate_parity(parity, msg_type);
  parity = accumulate_parity(parity, data_len);
  comms_port->write(START_BYTE);
  comms_port->write(msg_type);
  comms_port->write(data_len);
  if(data_len != 0)
    comms_port->write(data, data_len);
  comms_port->write(parity);
}

// XOR's all data bytes together to get a parity byte
uint8_t generate_parity(uint8_t *data, uint8_t len) {
  uint8_t parity = 0;
  for(int i = 0; i < len; i++) {
    parity = accumulate_parity(parity, data[i]);
  }
  return parity;
}

// XOR's a single byte with an existing parity byte
uint8_t accumulate_parity(uint8_t parity, uint8_t c) {
  return(parity^c);
}

// Read everything out of the serial buffer until either:
//  1) a message is received
//  2) the serial buffer is empty
bool receive_message(packet_t* packet_ptr) {
  while(comms_port->available()) {  // Keep reading as long as there's something in the serial buffer
    if(receive_byte(packet_ptr, comms_port->read())) {
      return true;  // If we get a coherent message, return true;
    }
  }
  return false; // If the buffer emptied without finding a message, return false
}

// Process a single received byte
// returns: true if a message was received, false otherwise
bool receive_byte(packet_t* packet_ptr, uint8_t c) {
  static packet_t received_packet;
  static uint8_t data_bytes_received;
  static uint8_t received_parity = 0;
  static enum parse_status_t parse_status = PARSE_UNINIT;
  switch(parse_status) {
    case PARSE_UNINIT:
    case PARSE_IDLE:    // Idle, waiting for a start byte to come through
      if(c == START_BYTE) {
        received_parity = START_BYTE;
        data_bytes_received = 0;
        parse_status = PARSE_TYPE;
      }
      break;
    case PARSE_TYPE:   // receiving the message type byte
      received_packet.msg_type = c;
      if(received_packet.msg_type >= INVALID_MSG_TYPE_CUTOFF) {
        parse_status = PARSE_IDLE;
        break;
      }
      received_parity = accumulate_parity(received_parity, c);
      parse_status = PARSE_LEN;
      break;
    case PARSE_LEN:    // receiving the data length byte
      received_packet.msg_len = c;
      received_parity = accumulate_parity(received_parity, c);
      if(received_packet.msg_len != 0) {
        parse_status = PARSE_DATA;
      }
      else {
        parse_status = PARSE_PARITY;
      }
      break;
    case PARSE_DATA:    // receiving data
      received_packet.data[data_bytes_received] = c;
      data_bytes_received++;
      received_parity = accumulate_parity(received_parity, c);
      if(data_bytes_received == received_packet.msg_len) {    // If we have received the correct number of bytes,
        parse_status = PARSE_PARITY;                          // the next byte will be the parity byte
      }
      break;
    case PARSE_PARITY:  // receiving the parity byte
      parse_status = PARSE_IDLE;
      if(received_parity == c) {  // If the parity is correct...
        memcpy(packet_ptr, &received_packet, PACKET_T_SIZE);  // Copy the packet data into the location specified by packet_ptr
        return true;
      }
      // If the parity was incorrect, the function will ignore the contents of the message and return false
      break;
  }
  return false;
}

// Sends a heartbeat message to the base station
void send_msg_heartbeat() {
  send_message(MSG_HEARTBEAT, 1, (uint8_t*)(&system_state));
}

// Sends a status message containing the passed string
void send_msg_status(const char* msg) {
  static char output_buffer[MAX_DATA_SIZE];
  uint8_t data_len = sprintf(output_buffer, "%s", msg);
  send_message(MSG_STATUS, data_len, (uint8_t*)output_buffer);
}

// Sends an acknowledgement message
void send_msg_ack(uint8_t ack_type) {
  send_message(MSG_ACK, 1, &ack_type);
}

// parses a received minefield message and returns the total number of mines that are present
uint16_t parse_msg_minefield(packet_t* packet) {
  return(*(uint16_t*)(packet->data));
}

// parses the lat/lon of a single mine, return the ID of the parsed mine
uint16_t parse_msg_mine(packet_t* packet, int32_t* lat_ptr, int32_t* lon_ptr, int32_t* esc_lat_ptr, int32_t* esc_lon_ptr) {
  *lat_ptr =     *((int32_t*)((packet->data) + 2));   // grab bytes 2,3,4,5, cast them to a single int32_t
  *lon_ptr =     *((int32_t*)((packet->data) + 6));   // grab bytes 6,7,8,9, cast them to a single int32_t
  *esc_lat_ptr = *((int32_t*)((packet->data) + 10));  // grab bytes 10,11,12,13, cast them to a single int32_t
  *esc_lat_ptr = *((int32_t*)((packet->data) + 14));  // grab bytes 14,15,16,17, cast them to a single int32_t
  return(*(uint16_t*)(packet->data));                 // grab bytes 0,1, cast them to a single uint16_t
}
