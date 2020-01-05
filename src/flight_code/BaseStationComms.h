#ifndef BASESTATIONCOMMS_H
#define BASESTATIONCOMMS_H

#include <Arduino.h>

#define START_BYTE 0xFE


/* PACKET STRUCTURE
 *  1) Start byte (0xFE)
 *  2) Data (message) type byte
 *  3) Data length byte
 *  4) Data bytes
 *  5) Parity byte
 */

enum message_types { MSG_HEARTBEAT=0,   // Heartbeat - sent to and received from the base station
                     MSG_STATUS=1,      // Status message - sending a string to the base station to update the user and debug
                     MSG_MINEFIELD=2,   // Minefield info - received from the base station. Basically just telling the drone how many mines to expect
                     MSG_MINE=3,        // Mine info - received from the base station. Coordinates for a single mine
                     MSG_TAKEOFF=4      // Signal from the base station to takeoff. Used as a trigger at the beggining of a run, or after a sandbag reload
};

void setup_comms(HardwareSerial*, uint32_t);
void send_message(uint8_t, uint8_t, uint8_t*);
uint8_t generate_parity(uint8_t*, uint8_t);

void send_msg_heartbeat();                // Sends a heartbeat message to the base station
void send_msg_status(const char*);             // Sends a status message containing the passed string
uint16_t parse_msg_minefield(uint8_t*);   // parses a received minefield message and returns the total number of mines that are present
void parse_msg_mine(uint8_t*, uint32_t*, uint32_t*);  // parses the lat/lon of a single mine

#endif
