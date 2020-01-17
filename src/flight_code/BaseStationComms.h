#ifndef BASESTATIONCOMMS_H
#define BASESTATIONCOMMS_H

#include <Arduino.h>

#define START_BYTE 0xFD
#define MAX_DATA_SIZE 128 // Maximum number of bytes that can be transmitted in the data field of a message


/* PACKET STRUCTURE
 *  1) Start byte (0xFD)          uint8_t
 *  2) Data (message) type byte   uint8_t
 *  3) Data length byte           uint8_t
 *  4) Data bytes                 uint8_t[]
 *  5) Parity byte                uint8_t
 */

// Simplified packet structure for received data
struct packet_t {
  uint8_t msg_type;
  uint8_t msg_len;
  uint8_t data[MAX_DATA_SIZE];
};

#define PACKET_T_SIZE sizeof(packet_t)

enum message_types { MSG_HEARTBEAT=0,   // Heartbeat - sent to and received from the base station
                     MSG_STATUS=1,      // Status message - sending a string to the base station to update the user and debug
                     MSG_MINEFIELD=2,   // Minefield info - received from the base station. Basically just telling the drone how many mines to expect
                     MSG_MINE=3,        // Mine info - received from the base station. Coordinates for a single mine
                     MSG_TAKEOFF=4,     // Signal from the base station to takeoff. Used as a trigger at the beggining of a run, or after a sandbag reload
                     MSG_ABORT=5,       // Signal from the base station to abort the current process and return to launch
                     MSG_ACK=6          // Acknowledgement packet - Confirms to the base station that a specific message type was received
};

enum parse_status_t { PARSE_UNINIT, PARSE_IDLE, PARSE_TYPE, PARSE_LEN, PARSE_DATA, PARSE_PARITY };

void setup_comms(HardwareSerial*, uint32_t);
void send_message(uint8_t, uint8_t, uint8_t*);
uint8_t generate_parity(uint8_t*, uint8_t);
uint8_t accumulate_parity(uint8_t, uint8_t);
bool receive_message(packet_t*);
bool receive_byte(packet_t*, uint8_t);

void send_msg_heartbeat();                // Sends a heartbeat message to the base station
void send_msg_status(char*);              // Sends a status message containing the passed string
void send_msg_ack(uint8_t);               // Sends an acknowledgement message
uint16_t parse_msg_minefield(packet_t*);  // parses a received minefield message and returns the total number of mines that are present
void parse_msg_mine(packet_t*, uint32_t*, uint32_t*);  // parses the lat/lon of a single mine

#endif
