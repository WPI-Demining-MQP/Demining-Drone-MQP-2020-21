#ifndef MAVLINKCOMMANDS_H
#define MAVLINKCOMMANDS_H

#include <Arduino.h>
#include "mavlink.h"

// Constants for sending messages
// Only sending to the Pixhawk, so these won't change
#define BUF_SIZE  288   // Max size of a Mavlink message
#define SYS_ID    255   // System ID
#define COMP_ID     1   // Component ID
#define TARGET_SYS  1   // Target system
#define TARGET_COMP 0   // Target component

// Flight characteristics
// Altitudes are in meters, relative to the takeoff point (home position)
#define OPERATING_ALT 6.096  // The altitude at which the drone will move from point to point (6.096 m = 20 ft)

// Type mask for setting a new position - see Mavlink docs for more info
// This configures the set position command to control position only (i.e. not velocity, yaw, or yaw velocity)
#define SET_POS_TYPE_MASK 0b110111111000

// Conditions for arm/disarm
#define ARM_CONDITION 1
#define DISARM_CONDITION 0

// Command status variable
// Each time we send a command to the Pixhawk, we need to wait for the appropriate response from the Pixhawk
// These variables track the response so we can reference it in the state machine
enum command_status_t {SENT, IN_PROGRESS, ACCEPTED, COMPLETED, REJECTED};
extern enum command_status_t command_status;

// Timeout variables
// These keep track of timeouts after commands are sent
// If a command is not acknowledged by the Pixhawk, it will be resent
#define NO_ACK_TIMEOUT 50
extern int32_t cmd_last_sent_time;  // time since last command was sent
extern MAV_CMD cmd_last_sent_type;  // type of the last command sent
extern float   cmd_last_sent_param; // parameter of the last command sent
extern uint16_t cmd_last_ack;       // type of the last command that was acknowledged

void setup_mavlink(HardwareSerial*, uint32_t);
int receive_mavlink(mavlink_message_t*, mavlink_status_t*);
void send_mavlink(mavlink_message_t*);
void set_command_status(mavlink_message_t*, mavlink_status_t*);
void arm(bool resend = false);
void disarm(bool resend = false);
void takeoff(bool resend = false);
void set_position_target(int32_t, int32_t);
void return_to_launch(bool resend = false);
void initiate_GPS_data(bool resend = false);
void initiate_GPS_fix_data(bool resend = false);
void request_home_location(bool resend = false);
void reset_home_location(bool resend = false);
bool check_timeouts();
void resend();

#endif
