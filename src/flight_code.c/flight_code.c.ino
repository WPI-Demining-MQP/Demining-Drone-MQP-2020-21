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

  // At the end of each loop, check for incoming messages
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
  // Pack a MAV_CMD_COMPONENT_ARM_DISARM message with the arm parameter set,
  // into a command_long message
  mavlink_msg_command_long_pack(SYS_ID, COMP_ID, &msg, TARGET_SYS, TARGET_COMP, MAV_CMD_COMPONENT_ARM_DISARM, 0, 1,0,0,0,0,0,0); // ARM
  send_mavlink(msg);
}

void disarm_drone() {
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(SYS_ID, COMP_ID, &msg, TARGET_SYS, TARGET_COMP, MAV_CMD_COMPONENT_ARM_DISARM, 0, 0,0,0,0,0,0,0); // DISARM
  send_mavlink(msg);
}
