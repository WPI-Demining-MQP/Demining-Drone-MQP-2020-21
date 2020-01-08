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

#define DROP_TARGET_ERROR_MARGIN 0.05     // Minimum acceptable error (in meters) from the target point when the drone is about to drop a payload
#define ESCAPE_TARGET_ERROR_MARGIN 0.5    // Minimum acceptable error (in meters) from the target point when the drone is escaping

// Default home latitude and longitude - these need to be updated by asking the Pixhawk for its home location
int32_t home_lat = 422756340, home_lon = -718030810;
int32_t current_lat, current_lon;   // Current lat/lon of the drone
uint32_t target_lat, target_lon;    // Target lat/lon for each movement

uint32_t last_fc_heartbeat_received = 0;    // Time that the last heartbeat message was received from the flight controller
uint32_t last_bs_heartbeat_received = 0;    // Time that the last heartbeat message was received from the base station
uint32_t last_bs_heartbeat_sent = 0;        // Time that the last heartbeat message was sent to the base station

// State machine states
// I'm sure we'll need to add more of these
enum state_t {DISARMED, BEGIN_TAKEOFF, TAKING_OFF, BEGIN_APPROACH, APPROACHING, DROP, BEGIN_ESCAPE, ESCAPING, BEGIN_RETURN_HOME, RETURNING_HOME, DONE, ABORT} state;

packet_t packet_in; // Global packet to use for incoming data from the base station
bool base_station_active = false; // Global to keep track of whether the base station is sending messages or not
bool in_flight = false;


void setup() {
  // enum for states of the setup state machine
  enum setup_state_t { WAIT_FOR_HEARTBEAT, WAIT_FOR_MINEFIELD, WAIT_FOR_MINES, WAIT_FOR_GPS_FIX, RESET_HOME_LOCATION, START_GPS_STREAM, WAIT_FOR_GPS_STREAM, REQUEST_HOME_LOCATION, WAIT_FOR_HOME_LOCATION, PLAN_PATH } setup_state = WAIT_FOR_HEARTBEAT;
  bool setup_complete = false;
  uint16_t num_mines_received = 0;
  node_t* head = NULL;    // head of the linked list that stores the incoming mine data
  mavlink_message_t msg_in;
  mavlink_status_t stat_in;
  
  DEBUG_PORT.begin(DEBUG_BAUD);
  
  setup_mavlink(&MAV_PORT, MAV_BAUD);   // initialize the mavlink connection
  setup_comms(&COMMS_PORT, COMMS_BAUD); // initialize the base station connection
  
  // Ensure we have a heartbeat from the Pixhawk before continuing to the state machine
  bool flight_controller_ready = false;
  while(!flight_controller_ready) {
    if(receive_mavlink(&msg_in, &stat_in)) {          // Interpret any incoming data
      if(msg_in.msgid == MAVLINK_MSG_ID_HEARTBEAT) {  // Check the message ID of an received message
        flight_controller_ready = true;
      }
    }
  }
  
  // Main setup state machine
  while(!setup_complete) {
    switch(setup_state) {
      case WAIT_FOR_HEARTBEAT:    // Wait for a heartbeat from the base station
        if(receive_message(&packet_in)) { // read the contents of the serial buffer looking for a message
          if(packet_in.msg_type == MSG_HEARTBEAT) {
            base_station_active = true;
            send_msg_status("Drone online");
            send_msg_status("Waiting for minefield data...");
            setup_state = WAIT_FOR_MINEFIELD;
          }
        }
        break;
      case WAIT_FOR_MINEFIELD:    // Wait for a minefield message (tells the drone how many mines to expect
        if(receive_message(&packet_in)) {
          if(packet_in.msg_type == MSG_MINEFIELD) {
            num_mines = parse_msg_minefield(&packet_in);
            send_msg_status("General minefield info received");
            send_msg_status("Waiting for mine locations...");
            setup_state = WAIT_FOR_MINES;
          }
        }
        break;
      case WAIT_FOR_MINES:        // Wait for all of the mine messages to arrive from the base station
        if(receive_message(&packet_in)) {
          if(packet_in.msg_type == MSG_MINE) {
            uint32_t lat,lon;
            parse_msg_mine(&packet_in, &lat, &lon);
            add_mine(&head, lat, lon);
            num_mines_received++;

            if(num_mines_received == num_mines) {
              char status_msg[MAX_DATA_SIZE];
              sprintf(status_msg, "Transfer complete - received data for %d mines", num_mines_received);
              send_msg_status(status_msg);
              send_msg_status("Waiting for GPS fix...");
              setup_state = WAIT_FOR_GPS_FIX;
            }
          }
        }
        break;
      case WAIT_FOR_GPS_FIX:      // Wait for a good GPS fix
        // TODO
  //      if(GPS gets a good fix) {
  //        send_msg_status("GPS fix acquired");
  //        setup_state = RESET_HOME_LOCATION
  //      }
      case RESET_HOME_LOCATION:   // Reset the flight controller's home location to the current location
        send_msg_status("Updating home location");
        reset_home_location();
        setup_state = START_GPS_STREAM;
        break;
      case START_GPS_STREAM:      // Initiate the stream of GPS data from the flight controller
        send_msg_status("Initiating GPS data-stream...");
        initiate_GPS_data();
        setup_state = WAIT_FOR_GPS_STREAM;
        break;
      case WAIT_FOR_GPS_STREAM:   // Wait for the stream of GPS data to begin
        if(receive_mavlink(&msg_in, &stat_in)) {          // Interpret any incoming data
          if(msg_in.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {  // Check the message ID of an received message
            current_lat = mavlink_msg_global_position_int_get_lat(&msg_in);
            current_lon = mavlink_msg_global_position_int_get_lon(&msg_in);
            send_msg_status("GPS data-stream active");
            setup_state = REQUEST_HOME_LOCATION;
          }
        }
        break;
      case REQUEST_HOME_LOCATION: // Ask the flight controller for it's home location (needed for path planning)
        send_msg_status("Updating home location...");
        request_home_location();
        setup_state = WAIT_FOR_HOME_LOCATION;
        break;
      case WAIT_FOR_HOME_LOCATION:  // Wait for the requested home location to arrive from the flight controller
        if(receive_mavlink(&msg_in, &stat_in)) {          // Interpret any incoming data
          if(msg_in.msgid == MAVLINK_MSG_ID_HOME_POSITION) {  // Check the message ID of an received message
            home_lat = mavlink_msg_home_position_get_latitude(&msg_in);
            home_lon = mavlink_msg_home_position_get_longitude(&msg_in);
            send_msg_status("Home location updated");
            setup_state = PLAN_PATH;
          }
        }
        break;
      case PLAN_PATH:             // Plan the drone's path through the minefield
        send_msg_status("Planning flight path...");
        plan_path(home_lat, home_lon, &head);
        send_msg_status("Flight path planning complete");
        setup_complete = true;    // Terminate the setup state machine
        break;
    }
    
    // Once we have heard a heartbeat from the base station, begin sending back a heartbeat every second (ish)
    if(base_station_active) {
      uint32_t cur_time = millis();
      if(cur_time - last_bs_heartbeat_sent >= 1000) {
        last_bs_heartbeat_sent = cur_time;
        send_msg_heartbeat();
      }
    }
  }
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
      target_lat = mines[mines_index].lat;
      target_lon = mines[mines_index].lon;
      set_position_target(target_lat, target_lon);    // Send a message to the Pixhawk telling it to move the drone
      state = APPROACHING;
      break;
    case APPROACHING:
      // Remain in this case until the drone is acceptably close to the target point
      if(dist_to(current_lat, current_lon, target_lat, target_lon) < DROP_TARGET_ERROR_MARGIN) {
        state = DROP;
      }
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
        get_escape_point(&target_lat, &target_lon);   // Calculate the target lat/lon. This function will place the target values in the variables pointed to by the function arguments
        set_position_target(target_lat, target_lon);  // Send the drone to the target point
        state = ESCAPING;
      }
      break;
    case ESCAPING:
      // Remain in this case until the drone is acceptable close to the target point
      if(dist_to(current_lat, current_lon, target_lat, target_lon) < ESCAPE_TARGET_ERROR_MARGIN) {
        state = BEGIN_APPROACH;
      }
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
        if(command_status == REJECTED) {
          char error_msg[MAX_DATA_SIZE];
          sprintf(error_msg, "ERROR: Flight controller rejected a command (ID#%d)", mavlink_msg_command_ack_get_command(&msg_in));
          send_msg_status(error_msg);
        }
        break;
      case MAVLINK_MSG_ID_HEARTBEAT:
        last_fc_heartbeat_received = millis();
        break;
      case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        current_lat = mavlink_msg_global_position_int_get_lat(&msg_in);
        current_lon = mavlink_msg_global_position_int_get_lon(&msg_in);
        break;
    }
  }

  uint32_t cur_time = millis(); // Grab the current time for use below
  
  // Check for incoming messages from the base station
  if(receive_message(&packet_in)) {
    switch(packet_in.msg_type) {
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
