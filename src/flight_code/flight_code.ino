/* Demining Autonomous System
 *  Drone subteam
 *  Flight code - to be run on the Teensy 3.5 onboard the drone
 *  
 *  Authors (Drone team members): Jessica McKenna, Joeseph Niski, Adam Santos, and Andrew VanOsten
 */

#include "mavlink.h"

// Constants for sending messages
// Only sending to the Pixhawk, so these won't change
#define SYS_ID    255   // System ID
#define COMP_ID   1     // Component ID
#define TARGET_SYS 1    // Target system
#define TARGET_COMP 0   // Target component

// Serial ports
// These are the hardware serial ports on the Teensy
#define DEBUG_PORT Serial   // Debugging port (USB)
#define MAV_PORT   Serial1  // MAVLink port
#define DEBUG_BAUD 115200
#define MAV_BAUD   19200

void setup() {
  DEBUG_PORT.begin(DEBUG_BAUD);
  MAV_PORT.begin(MAV_BAUD);
}

void loop() {
  // STATE MACHINE WILL GO HERE
  // TODO: state machine

  // At the end of each loop:
  // Check for incoming messages from the base station
  // TODO: [Insert function call here]
  
  // Check for incoming messages from the Pixhawk
  mavlink_message_t msg_in;
  mavlink_status_t stat_in;
  receive_mavlink(&msg_in, &stat_in);
}

// Reads from MAV_PORT until either:
//   1) the serial buffer is empty, or
//   2) a full message is found
// If (1), then the function returns 0. If any bytes remain, they will be stored statically
// If (2), then the function returns 1 and copies the message and status into the locations specified by the pointers
int receive_mavlink(mavlink_message_t *msg_ptr, mavlink_status_t *stat_ptr) {
  char byte_in;
  mavlink_message_t msg;
  mavlink_status_t stat;
  
  while(MAV_PORT.available()) {   // As long as there's serial data, read it
    byte_in = MAV_PORT.read();
    if(mavlink_parse_char(0, byte_in, &msg, &stat)) {   // If that byte completed the message, then handle it
      *msg_ptr = msg;   // Copy the received message into the location specified by msg_pointer
      *stat_ptr = stat; // Same with the status
      return 1;
    }
  }
  return 0;   // If we made it this far, then we didn't get a message this time
}

// Given a message, serializes it and sends it over MAV_PORT
void send_mavlink(mavlink_message_t msg) {
  uint8_t buf[100];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);   // Serialize the message - convert it into an array of bytes
  MAV_PORT.write(buf,len);  // Send the message
}

// Arms the drone
void arm_drone() {
  mavlink_message_t msg;
  // Pack a MAV_CMD_COMPONENT_ARM_DISARM command with the arm parameter set,
  // into a command_long message
  mavlink_msg_command_long_pack(SYS_ID, COMP_ID, &msg, TARGET_SYS, TARGET_COMP, MAV_CMD_COMPONENT_ARM_DISARM, 0, 1,0,0,0,0,0,0); // ARM
  send_mavlink(msg);
}

// Disarm the drone
void disarm_drone() {
  mavlink_message_t msg;
  // Pack a MAV_CMD_COMPONENT_ARM_DISARM command with the arm parameter cleared,
  // into a command_long message
  mavlink_msg_command_long_pack(SYS_ID, COMP_ID, &msg, TARGET_SYS, TARGET_COMP, MAV_CMD_COMPONENT_ARM_DISARM, 0, 0,0,0,0,0,0,0); // DISARM
  send_mavlink(msg);
}
