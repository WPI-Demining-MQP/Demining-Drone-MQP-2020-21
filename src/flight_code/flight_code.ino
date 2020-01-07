/* Demining Autonomous System
 *  Drone subteam
 *  Flight code - to be run on the Teensy 3.5 onboard the drone
 *  
 *  Authors (Drone team members): Jessica McKenna, Joeseph Niski, Adam Santos, and Andrew VanOsten
 */

#include "BaseStationComms.h"
#include "MavlinkCommands.h"
#include "PathPlan.h"

// Serial ports
// These are the hardware serial ports on the Teensy
#define DEBUG_PORT Serial   // Debugging port (USB)
#define MAV_PORT   Serial1  // MAVLink port
#define COMMS_PORT Serial2  // Base station comms port
#define DEBUG_BAUD 115200
#define MAV_BAUD   19200
#define COMMS_BAUD 19200

#define UNRESPONSIVE_SYSTEM_TIMEOUT 3000  // Maximum allowable time (in ms) without getting a heartbeat (flight controller or base station) before we raise red flags

// Default home latitude and longitude - these need to be updated by asking the Pixhawk for its home location
int32_t home_lat = 422756340, home_lon = -718030810;

uint32_t last_fc_heartbeat_received = 0;    // Time that the last heartbeat message was received from the flight controller
uint32_t last_bs_heartbeat_received = 0;    // Time that the last heartbeat message was received from the base station
uint32_t last_bs_heartbeat_sent = 0;        // Time that the last heartbeat message was sent to the base station

// State machine states
// I'm sure we'll need to add more of these
enum state_t {DISARMED, TAKEOFF, BEGIN_APPROACH, APPROACHING, DROP, BEGIN_ESCAPE, ESCAPING, BEGIN_RETURN_HOME, RETURNING_HOME, DONE, ABORT} state;

packet_t packet_in; // Global packet to use for incoming data from the base station
bool base_station_active = false; // Global to keep track of whether the base station is sending messages or not
bool in_flight = false;


void setup() {
  DEBUG_PORT.begin(DEBUG_BAUD);

  setup_mavlink(&MAV_PORT, MAV_BAUD);   // initialize the mavlink connection
  setup_comms(&COMMS_PORT, COMMS_BAUD); // initialize the base station connection

  bool flight_controller_ready = false, minefield_data_acquired = false, dGPS_ready = false;
  uint16_t num_mines_received = 0;
  node_t* head = NULL;    // head of the linked list that stores the incoming mine data
  mavlink_message_t msg_in;
  mavlink_status_t stat_in;
  uint8_t system_status = MAV_STATE_UNINIT;   // initial flight controller state is unknown
  
  while(!flight_controller_ready || !minefield_data_acquired || !dGPS_ready) {
    while(COMMS_PORT.available()) {
      if(receive_byte(&packet_in, COMMS_PORT.read())) {
        if(packet_in.msg_type == MSG_HEARTBEAT) {
          base_station_active = true;
        }
        else if(packet_in.msg_type == MSG_MINEFIELD) {
          num_mines = parse_msg_minefield(&packet_in);
        }
        else if(packet_in.msg_type == MSG_MINE && !minefield_data_acquired) {
          if(num_mines == 0) {  // We got a mine message before we were ready for it
            send_msg_status("ERROR: Mine data received without minefield data");
          }
          else {
            uint32_t lat,lon;
            parse_msg_mine(&packet_in, &lat, &lon);
            struct mine_t* new_mine = (struct mine_t*)malloc(MINE_T_SIZE);  // allocate some memory for the new mine data
            new_mine->lat = lat;
            new_mine->lon = lon;
            new_mine->isDetonated = false;  // assume the mine is live
            LL_add(&head, new_mine);  // Add the new mine to the linked list
            num_mines_received++;
            
            if(num_mines_received == num_mines) {
              minefield_data_acquired = true;
              send_msg_status("Minefield data transfer complete");
            }
          }
        }
        break;
      }
    }

    // Ensure we have a heartbeat from the Pixhawk before continuing to the state machine
    if(receive_mavlink(&msg_in, &stat_in)) {          // Interpret any incoming data
      if(msg_in.msgid == MAVLINK_MSG_ID_HEARTBEAT) {  // Check the message ID of an received message
        system_status = mavlink_msg_heartbeat_get_system_status(&msg_in);   // This message was a heartbeat - record the stated system status
        if(system_status == MAV_STATE_STANDBY) {
          flight_controller_ready = true;
        }
      }
    }

    // TODO: Determine fixed dGPS location before takeoff
    //       Wait for confirmation message about fixed dGPS location before proceeding
    //       Keep the base station up to date with the status of the dGPS

    // Send a heartbeat to the base station every second (ish)
    if(base_station_active) {
      uint32_t cur_time = millis();
      if(cur_time - last_bs_heartbeat_sent >= 1000) {
        last_bs_heartbeat_sent = cur_time;
        send_msg_heartbeat();
      }
    }
  }

  // Plan the drone's path through the minefield
  send_msg_status("Planning flight path...");
  plan_path(home_lat, home_lon, &head);
  send_msg_status("Flight path planning complete");
}

void loop() {
  // Main state machine - this handles the primary program flow
  switch(state) {
    case DISARMED:      // Disarmed, sitting on the ground
      if(in_flight) {
        state = BEGIN_TAKEOFF;
      }
      break;
    case BEGIN_TAKEOFF:
      arm();      // Arm the drone
      takeoff();  // Initiate takeoff
      state = TAKING_OFF;
      break;
    case TAKING_OFF:       // Actively taking off
      // listen for ACK response, and wait until complete
      if(command_status == ACCEPTED) {
        // takeoff complete, move on to mine approach
        command_status = COMPLETED; // Mark it as taken care of
        state = BEGIN_APPROACH;
      }
      break;
    case BEGIN_APPROACH:      // Approaching a mine
      set_position_target(mines[mines_index].lat, mines[mines_index].lon);    // Send a message to the Pixhawk telling it to move the drone
      state = APPROACHING;
      break;
    case APPROACHING:
      // if(close enough to target)
      //   state = DROP;
      break;
    case DROP:          //dropping the payload
      // TODO: trigger payload drop
      mines_index++;
      state = BEGIN_ESCAPE;
      break;
    case BEGIN_ESCAPE:  // Drone just dropped a payload, should now be running away
      if(mines_index % MINES_PER_RUN == 0 || mines_index >= num_mines) {
        state = BEGIN_RETURN_HOME;
      }
      else {
        // Generate a target point to escape to, and go there
        uint32_t target_lat, target_lon;  // Make a spot in memory for the target point
        get_escape_point(&target_lat, &target_lon);   // Calculate the target lat/lon. This function will place the target values in the variables pointed to by the function arguments
        set_position_target(target_lat, target_lon);  // Send the drone to the target point
        state = ESCAPING;
      }
      break;
    case ESCAPING:
      // TODO
      // if(close enough to target [escape point])
      //   state = BEGIN_APPROACH;
      break;
    case BEGIN_RETURN_HOME:   // Tells the Pixhawk to fly back to the launch point
      return_to_launch();
      state = RETURNING_HOME;
      break;
    case RETURNING_HOME:      // Drone is in the process of flying back to the launch point
      if(command_status == ACCEPTED) {
        command_status = COMPLETED;
        disarm();
        in_flight = false;
        if(mines_index == num_mines)
          state = DONE;
        else
          state = DISARMED;
      }
      break;
    case ABORT:         // User clicks the abort button and the drone needs to return to base
      state = BEGIN_RETURN_HOME;  // Send it home.
      break;
    case DONE:          // Mission completed, state will remain here
      send_msg_status("Mission complete - drone disabled");
      break;
  }
  
  // Check for incoming messages from the Pixhawk
  mavlink_message_t msg_in;
  mavlink_status_t stat_in;
  if(receive_mavlink(&msg_in, &stat_in)) {
    switch(msg_in.msgid) {
      case MAVLINK_MSG_ID_COMMAND_ACK:
        set_command_status(&msg_in, &stat_in);
        break;
      case MAVLINK_MSG_ID_HEARTBEAT:
        last_fc_heartbeat_received = millis();
        break;
    }
  }

  uint32_t cur_time = millis(); // Grab the current time for use below
  
  // Check for incoming messages from the base station
  if(receive_message(&packet_in)) {
    switch(packet_in.msg_id) {
      case MSG_HEARTBEAT:
        base_station_active = true;
        last_bs_heartbeat_received = cur_time;
        break;
      case MSG_TAKEOFF:
        in_flight = true;
        break;
      case MSG_ABORT:
        state = ABORT;  // If we get an abort message, stop the current process and go to the abort case
    }
  }
  
  if(cur_time - last_fc_heartbeat_received > UNRESPONSIVE_SYSTEM_TIMEOUT) {
    // It's been too long since the Pixhawk has sent a heartbeat - something has gone wrong
    send_msg_status("ERROR: flight controller is unresponsive");
  }
  if(cur_time - last_bs_heartbeat_received > UNRESPONSIVE_SYSTEM_TIMEOUT) {
    // It's been too long since the base station has sent a heartbeat - something has gone wrong
    base_station_active = false;
  }

  // Ensure that the last Mavlink message sent to the Pixhawk was acknowledged within the timeout
  // If not, the command will be resent
  check_timeouts();
}
