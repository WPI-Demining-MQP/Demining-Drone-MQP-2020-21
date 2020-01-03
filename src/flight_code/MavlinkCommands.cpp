#include "MavlinkCommands.h"

// Serial port for mavlink to use
// Defaults to Serial1, but redefined by the setup_mavlink() call
HardwareSerial* mav_port = &Serial1;

// Command status variable
enum command_status_t command_status;

int32_t cmd_last_sent_time = -1;  // time since last command was sent
MAV_CMD cmd_last_sent_type;       // type of the last command sent
float cmd_last_sent_param;        // parameter of the last command sent

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

// Given a pointer to a message, this function converts it to an array of bytes sends it over MAV_PORT
void send_mavlink(mavlink_message_t* msg_ptr) {
  uint8_t buf[BUF_SIZE];  // Memory location for the message
  uint16_t len = mavlink_msg_to_send_buffer(buf, msg_ptr);   // Serialize the message - convert it into an array of bytes
  mav_port->write(buf,len);  // Send the message
}

// Takes a received ACK message and updates the command_status variable
void set_command_status(mavlink_message_t *msg, mavlink_status_t *stat) {
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

// Arms the drone
void arm(bool resend) {
  mavlink_message_t msg;
  static uint8_t confirmation = 0;
  if(resend == true)
    confirmation++;
  else
    confirmation = 0;
  // Pack a MAV_CMD_COMPONENT_ARM_DISARM command with the arm parameter set,
  // into a command_long message
  mavlink_msg_command_long_pack(SYS_ID, COMP_ID, &msg, TARGET_SYS, TARGET_COMP, MAV_CMD_COMPONENT_ARM_DISARM, confirmation, ARM_CONDITION,0,0,0,0,0,0); // ARM
  send_mavlink(&msg);
  cmd_last_sent_time = millis();
  cmd_last_sent_type = MAV_CMD_COMPONENT_ARM_DISARM;
  cmd_last_sent_param = ARM_CONDITION;
  command_status = SENT;
}

// Disarm the drone
void disarm(bool resend) {
  mavlink_message_t msg;
  static uint8_t confirmation = 0;
  if(resend == true)
    confirmation++;
  else
    confirmation = 0;
  // Pack a MAV_CMD_COMPONENT_ARM_DISARM command with the arm parameter cleared,
  // into a command_long message
  mavlink_msg_command_long_pack(SYS_ID, COMP_ID, &msg, TARGET_SYS, TARGET_COMP, MAV_CMD_COMPONENT_ARM_DISARM, confirmation, DISARM_CONDITION,0,0,0,0,0,0); // DISARM
  send_mavlink(&msg);
  cmd_last_sent_time = millis();
  cmd_last_sent_type = MAV_CMD_COMPONENT_ARM_DISARM;
  cmd_last_sent_param = DISARM_CONDITION;
  command_status = SENT;
}

// Takeoff
// Takes off and climbs to OPERATING_ALT
// Maintains lat/lon position
void takeoff(bool resend) {
  mavlink_message_t msg;
  static uint8_t confirmation = 0;
  if(resend == true)
    confirmation++;
  else
    confirmation = 0;
  // Pack a MAV_CMD_NAV_TAKEOFF command with the altitude parameter set,
  // into a command_long message
  mavlink_msg_command_long_pack(SYS_ID, COMP_ID, &msg, TARGET_SYS, TARGET_COMP, MAV_CMD_NAV_TAKEOFF, confirmation, 0,0,0,0,0,0,OPERATING_ALT); // TAKEOFF
  send_mavlink(&msg);
  cmd_last_sent_time = millis();
  cmd_last_sent_type = MAV_CMD_NAV_TAKEOFF;
  command_status = SENT;
}

void set_position_target(uint32_t target_lat, uint32_t target_lon) {
  mavlink_message_t msg;
  mavlink_msg_set_position_target_global_int_pack(SYS_ID, COMP_ID, &msg, millis(), TARGET_SYS, TARGET_COMP, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, SET_POS_TYPE_MASK, target_lat, target_lon, OPERATING_ALT, 0,0,0, 0,0,0, 0,0);
  send_mavlink(&msg);
}

void check_timeouts() {
  int32_t cur_time = millis();
  if(command_status == SENT) {
    if(cur_time - cmd_last_sent_time >= NO_ACK_TIMEOUT) {
      resend();  // if the command has not been acknowledged, resend it
    }
  }
}

void resend() {
  if(cmd_last_sent_type == MAV_CMD_NAV_TAKEOFF) {
    takeoff(true);
  }
  else if(cmd_last_sent_type == MAV_CMD_COMPONENT_ARM_DISARM) {
    if(cmd_last_sent_param == ARM_CONDITION) {
      arm(true);
    }
    else if(cmd_last_sent_param == DISARM_CONDITION) {
      disarm(true);
    }
  }
}
