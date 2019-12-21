#ifndef MAVLINKCOMMANDS_H
#define MAVLINKCOMMANDS_H

#include <Arduino.h>
#include "mavlink.h"

// Constants for sending messages
// Only sending to the Pixhawk, so these won't change
#define SYS_ID    255   // System ID
#define COMP_ID     1   // Component ID
#define TARGET_SYS  1   // Target system
#define TARGET_COMP 0   // Target component

// Flight characteristics
// Altitudes are relative to the takeoff point (home position)
#define OPERATING_ALT 20  // The altitude at which the drone will move from point to point
#define DROP_ALT      20  // The altitude at which the drone will drop payloads

// Command status variable
// Each time we send a command to the Pixhawk, we need to wait for the appropriate response from the Pixhawk
// These variables track the response so we can reference it in the state machine
enum command_status_t {ACCEPTED, IN_PROGRESS, COMPLETED};
extern enum command_status_t command_status;

void setup_mavlink(HardwareSerial*);
int receive_mavlink(mavlink_message_t*, mavlink_status_t*);
void send_mavlink(mavlink_message_t);
void set_globals(mavlink_message_t*, mavlink_status_t*);
void arm();
void disarm();
void takeoff();

#endif
