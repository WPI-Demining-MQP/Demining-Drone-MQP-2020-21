/* Demining Autonomous System
 *  Drone subteam
 *  Flight code - to be run on the Teensy 3.5 onboard the drone
 *  
 *  Authors (Drone team members): Jessica McKenna, Joeseph Niski, Adam Santos, and Andrew VanOsten
 */

#include "BaseStationComms.h"
#include "MavlinkCommands.h"
#include "Minefield.h"

// Serial ports
// These are the hardware serial ports on the Teensy
#define DEBUG_PORT Serial   // Debugging port (USB)
#define MAV_PORT   Serial1  // MAVLink port
#define COMMS_PORT Serial2  // Base station comms port
#define DEBUG_BAUD 115200
#define MAV_BAUD   19200
#define COMMS_BAUD 57600

#define OK_FIX_TYPE GPS_FIX_TYPE_DGPS  // Acceptable GPS fix type
#define UNRESPONSIVE_SYSTEM_TIMEOUT 5000  // Maximum allowable time (in ms) without getting a heartbeat (flight controller or base station) before we raise red flags

#define DROP_TARGET_ERROR_MARGIN 0.05     // Minimum acceptable error (in meters) from the target point when the drone is about to drop a payload
#define ESCAPE_TARGET_ERROR_MARGIN 0.5    // Minimum acceptable error (in meters) from the target point when the drone is escaping

// Default home latitude and longitude - these need to be updated by asking the Pixhawk for its home location
int32_t home_lat = 422756340, home_lon = -718030810;
int32_t current_lat, current_lon;   // Current lat/lon of the drone
int32_t current_relative_alt;       // Current altitude relative to takeoff point
int32_t target_lat, target_lon;     // Target lat/lon for each movement
uint8_t fix_status = GPS_FIX_TYPE_NO_FIX; // GPS fix type - uses Mavlink's GPS_FIX_TYPE enum
uint8_t mav_state = MAV_STATE_UNINIT;

uint32_t last_fc_heartbeat_received = 0;    // Time that the last heartbeat message was received from the flight controller
uint32_t last_bs_heartbeat_received = 0;    // Time that the last heartbeat message was received from the base station
uint32_t last_bs_heartbeat_sent = 0;        // Time that the last heartbeat message was sent to the base station

// State machine states
// I'm sure we'll need to add more of these
enum state_t {DISARMED, BEGIN_ARM, WAIT_FOR_ARM, BEGIN_TAKEOFF, WAIT_FOR_TAKEOFF_ACK, TAKING_OFF, BEGIN_APPROACH, APPROACHING, DROP, BEGIN_ESCAPE, ESCAPING, BEGIN_RETURN_HOME, RETURNING_HOME, WAIT_FOR_DISARM, BEGIN_DISARM, WAIT_FOR_DISARM_ACK, DONE, ABORT} state;

packet_t packet_in; // Global packet to use for incoming data from the base station
bool base_station_active = false; // Global to keep track of whether the base station is sending messages or not
bool in_flight = false;
uint32_t last_error_print_time = 0;


void setup() {
  // enum for states of the setup state machine
  enum setup_state_t { START_GPS_FIX_STREAM, WAIT_FOR_GPS_FIX_DATA, WAIT_FOR_GPS_FIX, RESET_HOME_LOCATION, WAIT_FOR_RESET_HOME_ACK, START_GPS_STREAM, WAIT_FOR_GPS_STREAM, REQUEST_HOME_LOCATION, WAIT_FOR_HOME_LOCATION, SEND_HOME_LOCATION, WAIT_FOR_MINEFIELD, WAIT_FOR_MINES, ERROR_LOITER } setup_state = START_GPS_FIX_STREAM;
  bool setup_complete = false;
  bool GPS_position_received = false, fix_stream_active = false, home_location_updated = false;
  uint16_t num_mines_received = 0;
  mavlink_message_t msg_in;
  mavlink_status_t stat_in;
  
  DEBUG_PORT.begin(DEBUG_BAUD);
  
  setup_mavlink(&MAV_PORT, MAV_BAUD);   // initialize the mavlink connection
  setup_comms(&COMMS_PORT, COMMS_BAUD); // initialize the base station connection

  // Ensure we have a heartbeat from the base station before continuing
  while(!base_station_active) {
    if(receive_message(&packet_in)) { // read the contents of the serial buffer looking for a message
      if(packet_in.msg_type == MSG_HEARTBEAT) {
        base_station_active = true;
        last_bs_heartbeat_received = millis();
        send_msg_status("Drone online, waiting for flight controller");
      }
    }
  }

  // Wait until the flight controller has initiallized fully
  bool flight_controller_ready = false;
  while(!flight_controller_ready) {
    if(receive_mavlink(&msg_in, &stat_in)) {          // Interpret any incoming data
      if(msg_in.msgid == MAVLINK_MSG_ID_HEARTBEAT) {  // Check the message ID of an received message
        mav_state = mavlink_msg_heartbeat_get_system_status(&msg_in);
        if(mav_state == MAV_STATE_STANDBY) {
          flight_controller_ready = true;
          last_fc_heartbeat_received = millis();
        }
      }
    }
  }

  system_state = SYS_STATE_SETUP;
  // Main setup state machine
  while(!setup_complete) {
    switch(setup_state) {
      case START_GPS_FIX_STREAM:
        send_msg_status("Initiating GPS fix data-stream...");
        initiate_GPS_fix_data();
        setup_state = WAIT_FOR_GPS_FIX_DATA;
        break;
      case WAIT_FOR_GPS_FIX_DATA:
        if(fix_stream_active) {
          send_msg_status("GPS fix data-stream active");
          send_msg_status("Waiting for GPS fix (this may take a few minutes)...");
          setup_state = WAIT_FOR_GPS_FIX;
        }
      case WAIT_FOR_GPS_FIX:      // Wait for a good GPS fix
        if(fix_status >= OK_FIX_TYPE && fix_status <= GPS_FIX_TYPE_RTK_FIXED) {
          send_msg_status("GPS fix acquired");
          setup_state = RESET_HOME_LOCATION;
        }
        break;
      case RESET_HOME_LOCATION:   // Reset the flight controller's home location to the current location
        send_msg_status("Resetting home location");
        reset_home_location();
        setup_state = WAIT_FOR_RESET_HOME_ACK;
        break;
      case WAIT_FOR_RESET_HOME_ACK: // Wait for the flight controller to acknowledge the reset of the home location
        if(command_status == ACCEPTED && cmd_last_ack == MAV_CMD_DO_SET_HOME) {
          command_status = COMPLETED;
          send_msg_status("Home location reset");
          setup_state = START_GPS_STREAM;
        }
        else if(command_status == REJECTED && cmd_last_ack == MAV_CMD_DO_SET_HOME) {
          command_status = COMPLETED;
          send_msg_status("Home location reset was rejected");
          setup_state = ERROR_LOITER;
        }
        break;
      case START_GPS_STREAM:      // Initiate the stream of GPS data from the flight controller
        send_msg_status("Initiating GPS data-stream...");
        initiate_GPS_data();
        setup_state = WAIT_FOR_GPS_STREAM;
        break;
      case WAIT_FOR_GPS_STREAM:   // Wait for the stream of GPS data to begin
        if(GPS_position_received) {
          send_msg_status("GPS data-stream active");
          setup_state = REQUEST_HOME_LOCATION;
        }
        break;
      case REQUEST_HOME_LOCATION: // Ask the flight controller for it's home location (needed for path planning)
        send_msg_status("Updating home location...");
        request_home_location();
        setup_state = WAIT_FOR_HOME_LOCATION;
        break;
      case WAIT_FOR_HOME_LOCATION:  // Wait for the requested home location to arrive from the flight controller
        if(home_location_updated) {
          send_msg_status("Home location updated");
          setup_state = SEND_HOME_LOCATION;
        }
        break;
      case SEND_HOME_LOCATION:    // Send the newly updated home location back to the base station for path planning
        send_msg_status("Sending home location");
        send_msg_home(home_lat, home_lon);
        setup_state = WAIT_FOR_MINEFIELD;
        break;
      case WAIT_FOR_MINEFIELD:    // Wait for a minefield message (tells the drone how many mines to expect
        // num_mines is initialized to 0
        // When a minefield message is received, it's value is updated
        if(num_mines) {
          setup_state = WAIT_FOR_MINES;
        }
        break;
      case WAIT_FOR_MINES:        // Wait for all of the mine messages to arrive from the base station
        if(num_mines_received == num_mines) {
          char status_msg[MAX_DATA_SIZE];
          sprintf(status_msg, "Transfer complete - received data for %d mines", num_mines_received);
          send_msg_status(status_msg);
          setup_complete = true;
        }
        break;
      case ERROR_LOITER:
        break;
    }
    
    // Send a heartbeat to the base station every second (ish)
    uint32_t cur_time = millis();
    if(cur_time - last_bs_heartbeat_sent >= 1000) {
      last_bs_heartbeat_sent = cur_time;
      send_msg_heartbeat();
    }

    // Check for incoming messages from the base station
    if(receive_message(&packet_in)) {
      switch(packet_in.msg_type) {
        case MSG_HEARTBEAT:
          base_station_active = true;
          last_bs_heartbeat_received = cur_time;
          break;
        case MSG_MINEFIELD:
          num_mines = parse_msg_minefield(&packet_in);
          send_msg_status("General minefield info received");
          send_msg_status("Waiting for mine locations...");
          send_msg_ack(MSG_MINEFIELD);
          setup_state = WAIT_FOR_MINES;
          break;
        case MSG_MINE:
          // Each incoming mine as a zero-based index. Only accept this incoming mine if it has an index that we haven't received yet, and we're expecting more mines to come in
          if(parse_msg_mine(&packet_in, &mines[num_mines_received]) == num_mines_received && num_mines_received < num_mines) {
            num_mines_received++;
            send_msg_ack(MSG_MINE);
          }
          break;
      }
    }

    // Check for incoming messages from the flight controller
    mavlink_message_t msg_in;
    mavlink_status_t stat_in;
    if(receive_mavlink(&msg_in, &stat_in)) {
//      char message[128];
//      sprintf(message, "FC_MSG#%d", msg_in.msgid);
//      send_msg_status(message);
      switch(msg_in.msgid) {
        case MAVLINK_MSG_ID_COMMAND_ACK:
          char msg[128];
          sprintf(msg, "ACK (ID#%d) result=%d", mavlink_msg_command_ack_get_command(&msg_in), mavlink_msg_command_ack_get_result(&msg_in));
          send_msg_status(msg);
          cmd_last_ack = mavlink_msg_command_ack_get_command(&msg_in);
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
          GPS_position_received = true;
          current_lat = mavlink_msg_global_position_int_get_lat(&msg_in);
          current_lon = mavlink_msg_global_position_int_get_lon(&msg_in);
          current_relative_alt = mavlink_msg_global_position_int_get_relative_alt(&msg_in);
          break;
        case MAVLINK_MSG_ID_HOME_POSITION:
          home_location_updated = true;
          home_lat = mavlink_msg_home_position_get_latitude(&msg_in);
          home_lon = mavlink_msg_home_position_get_longitude(&msg_in);
          break;
        case MAVLINK_MSG_ID_STATUSTEXT:
          char text[51];
          mavlink_msg_statustext_get_text(&msg_in, text);
          send_msg_status(text);
          break;
        case MAVLINK_MSG_ID_PARAM_VALUE:
          char param_id[17];
          mavlink_msg_param_value_get_param_id(&msg_in, param_id);
          break;
        case MAVLINK_MSG_ID_GPS_RAW_INT:
          fix_status = mavlink_msg_gps_raw_int_get_fix_type(&msg_in);
          fix_stream_active = true;
          break;
      }
    }

    if((cur_time - last_fc_heartbeat_received) > UNRESPONSIVE_SYSTEM_TIMEOUT) {
      // It's been too long since the Pixhawk has sent a heartbeat - something has gone wrong
      if(millis() - last_error_print_time > 1000) {
        send_msg_status("ERROR: flight controller is unresponsive");
        last_error_print_time = millis();
      }
    }
    if((cur_time - last_bs_heartbeat_received) > UNRESPONSIVE_SYSTEM_TIMEOUT) {
      // It's been too long since the base station has sent a heartbeat - something has gone wrong
      base_station_active = false;
      DEBUG_PORT.println("ERROR: Base station connection lost");
      //delay(1000);  // Nothing we can do without the base station. We're not in the air yet, so we'll just wait and see if it reconnects
    }
  }

  system_state = SYS_STATE_STANDBY;
  send_msg_status("SETUP COMPLETE");
}

void loop() {
  // Main state machine - this handles the primary program flow
  switch(state) {
    case DISARMED:      // Disarmed, sitting on the ground
      system_state = SYS_STATE_STANDBY;
      if(in_flight) {
        state = BEGIN_ARM;
      }
      break;
    case BEGIN_ARM:
      system_state = SYS_STATE_ACTIVE;
      send_msg_status("Arming");
      arm();      // Arm the drone
      state = WAIT_FOR_ARM;
      break;
    case WAIT_FOR_ARM:
      if(command_status == ACCEPTED && cmd_last_ack == MAV_CMD_COMPONENT_ARM_DISARM) {
        send_msg_status("Arm command accepted");
        command_status = COMPLETED;
        state = BEGIN_TAKEOFF;
      }
      else if(command_status == REJECTED && cmd_last_ack == MAV_CMD_COMPONENT_ARM_DISARM) {
        send_msg_status("Arm command rejected");
        command_status = COMPLETED;
        state = DISARMED;
      }
      break;
    case BEGIN_TAKEOFF:
      send_msg_status("Taking off");
      takeoff();  // Initiate takeoff
      state = WAIT_FOR_TAKEOFF_ACK;
      break;
    case WAIT_FOR_TAKEOFF_ACK:      // Actively taking off
      // listen for ACK response, and wait until complete
      if(command_status == ACCEPTED && cmd_last_ack == MAV_CMD_NAV_TAKEOFF) {
        send_msg_status("Takeoff command accepted");
        // takeoff complete, move on to mine approach
        command_status = COMPLETED; // Mark it as taken care of
        state = TAKING_OFF;
      }
      else if(command_status == REJECTED && cmd_last_ack == MAV_CMD_NAV_TAKEOFF) {
        send_msg_status("Takeoff command rejected");
        // takeoff complete, move on to mine approach
        command_status = COMPLETED; // Mark it as taken care of
        in_flight = false;
        state = DISARMED;
      }
      break;
    case TAKING_OFF:
      {
      double alt_ratio = double(current_relative_alt)/OPERATING_ALT;
      if(alt_ratio >= 0.95 && alt_ratio <= 1.05) {    // if we're within 5% of the target altitude...
        send_msg_status("Target altitude reached");
//        state = BEGIN_APPROACH;
        state = BEGIN_RETURN_HOME;
      }
      break;
      }
    case BEGIN_APPROACH:      // Approaching a mine
      send_msg_status("Beginning approach");
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
      send_msg_status("Dropping");
      // TODO: trigger payload drop
      mines_index++;
      state = BEGIN_ESCAPE;
      break;
    case BEGIN_ESCAPE:  // Drone just dropped a payload, should now be running away
      if(mines_index % MINES_PER_RUN == 0 || mines_index >= num_mines) {
        state = BEGIN_RETURN_HOME;
      }
      else {
        set_position_target(mines[mines_index-1].escape_lat, mines[mines_index-1].escape_lon);  // Send the drone to the escape point
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
      if(command_status == ACCEPTED && cmd_last_ack == MAV_CMD_NAV_RETURN_TO_LAUNCH) {
        command_status = COMPLETED;
        state = WAIT_FOR_DISARM;
      }
      break;
    case WAIT_FOR_DISARM:
      if(mav_state == MAV_STATE_STANDBY) {
        in_flight = false;
        if(mines_index == num_mines)
          state = DONE;
        else
          state = DISARMED;
      }
      break;
//    case BEGIN_DISARM:        // Sends the disarm command
//      send_msg_status("Disarming");
//      disarm();
//      state = WAIT_FOR_DISARM;
//      break;
//    case WAIT_FOR_DISARM_ACK:     // Waits for an acknowledgement of the disarm command
//      if(command_status == ACCEPTED && cmd_last_ack == MAV_CMD_COMPONENT_ARM_DISARM) {
//        send_msg_status("Disarm command accepted");
//        
//        if(mines_index == num_mines)
//          state = DONE;
//        else
//          state = DISARMED;
//        
//        in_flight = false;
//        command_status = COMPLETED;
//      }
//      else if(command_status == REJECTED && cmd_last_ack == MAV_CMD_COMPONENT_ARM_DISARM) {
//        send_msg_status("Disarm command rejected");
//        command_status = COMPLETED;
//      }
//      break;
    case ABORT:         // User clicks the abort button and the drone needs to return to base
      send_msg_status("ABORTING MISSION");
      system_state = SYS_STATE_EMERGENCY;
      state = BEGIN_RETURN_HOME;  // Send it home.
      break;
    case DONE:          // Mission completed, state will remain here
      break;
  }
  
  // Check for incoming messages from the Pixhawk
  mavlink_message_t msg_in;
  mavlink_status_t stat_in;
  if(receive_mavlink(&msg_in, &stat_in)) {
    char message[128];
//    sprintf(message, "FC_MSG#%d", msg_in.msgid);
//    send_msg_status(message);
    switch(msg_in.msgid) {
      case MAVLINK_MSG_ID_COMMAND_ACK:
        cmd_last_ack = mavlink_msg_command_ack_get_command(&msg_in);
        sprintf(message, "FC_MSG_ACK#%d, result=%d", cmd_last_ack, mavlink_msg_command_ack_get_result(&msg_in));
        send_msg_status(message);
        set_command_status(&msg_in, &stat_in);
        if(command_status == REJECTED) {
          char error_msg[MAX_DATA_SIZE];
          sprintf(error_msg, "ERROR: Flight controller rejected a command (ID#%d)", mavlink_msg_command_ack_get_command(&msg_in));
          send_msg_status(error_msg);
        }
        break;
      case MAVLINK_MSG_ID_HEARTBEAT:
        last_fc_heartbeat_received = millis();
        mav_state = mavlink_msg_heartbeat_get_system_status(&msg_in);
        break;
      case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        current_lat = mavlink_msg_global_position_int_get_lat(&msg_in);
        current_lon = mavlink_msg_global_position_int_get_lon(&msg_in);
        current_relative_alt = mavlink_msg_global_position_int_get_relative_alt(&msg_in);
        break;
      case MAVLINK_MSG_ID_STATUSTEXT:
        char status_text[128];
        mavlink_msg_statustext_get_text(&msg_in, status_text);
        send_msg_status(status_text);
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
        send_msg_ack(MSG_TAKEOFF);
        in_flight = true;
        break;
      case MSG_ABORT:
        send_msg_ack(MSG_TAKEOFF);
        state = ABORT;  // If we get an abort message, stop the current process and go to the abort case
    }
  }

  // Send a heartbeat to the base station every second (ish)
  if(cur_time - last_bs_heartbeat_sent >= 1000) {
    last_bs_heartbeat_sent = cur_time;
    send_msg_heartbeat();
  }
  
  if(cur_time - last_fc_heartbeat_received > UNRESPONSIVE_SYSTEM_TIMEOUT) {
    // It's been too long since the Pixhawk has sent a heartbeat - something has gone wrong
    if(millis() - last_error_print_time > 1000) {
      send_msg_status("ERROR: flight controller is unresponsive");
      last_error_print_time = millis();
    }
  }
  if(cur_time - last_bs_heartbeat_received > UNRESPONSIVE_SYSTEM_TIMEOUT) {
    // It's been too long since the base station has sent a heartbeat - something has gone wrong
    base_station_active = false;
  }

  // Ensure that the last Mavlink message sent to the Pixhawk was acknowledged within the timeout
  // If not, the command will be resent
  if(check_timeouts()) {
    char message[32];
    sprintf(message, "Resent (ID#%d)", cmd_last_sent_type);
    send_msg_status(message);
  }
}
