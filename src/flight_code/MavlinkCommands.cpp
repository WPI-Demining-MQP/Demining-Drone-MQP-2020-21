#include "MavlinkCommands.h"

// Serial port for mavlink to use
// Defaults to Serial1, but redefined by the setup_mavlink() call
HardwareSerial* mav_port = &Serial1;

// Command status variable
enum command_status_t command_status;

void setup_mavlink(HardwareSerial *serial_ptr) {
  mav_port = serial_ptr;
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
  
  while(mav_port->available()) {   // As long as there's serial data, read it
    byte_in = mav_port->read();
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
  mav_port->write(buf,len);  // Send the message
}

// Takes a received message and updates global status variables
void set_globals(mavlink_message_t *msg, mavlink_status_t *stat) {
  uint8_t result = mavlink_msg_command_ack_get_result(msg);
  if(result == MAV_RESULT_ACCEPTED) {
      command_status = ACCEPTED;
  }
  else if(result == MAV_RESULT_TEMPORARILY_REJECTED ||
          result == MAV_RESULT_DENIED ||
          result == MAV_RESULT_UNSUPPORTED ||
          result == MAV_RESULT_FAILED) {
    // The Pixhawk returned an error - send something back to the base station
    // TODO: Send and error message to the base station
  }
  else if(result == MAV_RESULT_IN_PROGRESS) {
    command_status = IN_PROGRESS;
  }
}

// TODO: Handle comfirmation field in arm, disarm, and takeoff functions

// Arms the drone
void arm() {
  mavlink_message_t msg;
  // Pack a MAV_CMD_COMPONENT_ARM_DISARM command with the arm parameter set,
  // into a command_long message
  mavlink_msg_command_long_pack(SYS_ID, COMP_ID, &msg, TARGET_SYS, TARGET_COMP, MAV_CMD_COMPONENT_ARM_DISARM, 0, 1,0,0,0,0,0,0); // ARM
  send_mavlink(msg);
}

// Disarm the drone
void disarm() {
  mavlink_message_t msg;
  // Pack a MAV_CMD_COMPONENT_ARM_DISARM command with the arm parameter cleared,
  // into a command_long message
  mavlink_msg_command_long_pack(SYS_ID, COMP_ID, &msg, TARGET_SYS, TARGET_COMP, MAV_CMD_COMPONENT_ARM_DISARM, 0, 0,0,0,0,0,0,0); // DISARM
  send_mavlink(msg);
}

// Takeoff
// Takes off and climbs to OPERATING_ALT
// Maintains lat/lon position
void takeoff() {
  mavlink_message_t msg;
  // Pack a MAV_CMD_NAV_TAKEOFF command with the altitude parameter set,
  // into a command_long message
  mavlink_msg_command_long_pack(SYS_ID, COMP_ID, &msg, TARGET_SYS, TARGET_COMP, MAV_CMD_NAV_TAKEOFF, 0, 0,0,0,0,0,0,OPERATING_ALT); // TAKEOFF
  send_mavlink(msg);
}
