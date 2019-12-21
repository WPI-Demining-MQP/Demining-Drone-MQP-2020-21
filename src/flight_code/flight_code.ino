/* Demining Autonomous System
 *  Drone subteam
 *  Flight code - to be run on the Teensy 3.5 onboard the drone
 *  
 *  Authors (Drone team members): Jessica McKenna, Joeseph Niski, Adam Santos, and Andrew VanOsten
 */

#include "MavlinkCommands.h"
#include "PathPlan.h"

// Serial ports
// These are the hardware serial ports on the Teensy
#define DEBUG_PORT Serial   // Debugging port (USB)
#define MAV_PORT   Serial1  // MAVLink port
#define DEBUG_BAUD 115200
#define MAV_BAUD   19200

// Default home latitude and longitude - these need to be updated by asking the Pixhawk for its home location
int32_t home_lat = 422756340, home_lon = -718030810;

// State machine states
// I'm sure we'll need to add more of these
enum state_t {DISARMED, ARMED, TAKEOFF, LAND, APPROACH, READY_TO_DROP, DROP, ESCAPE, ABORT, RETURN_HOME} state;


void setup() {
  DEBUG_PORT.begin(DEBUG_BAUD);
  MAV_PORT.begin(MAV_BAUD);

  setup_mavlink(&MAV_PORT);

  // Determine fixed dGPS location before takeoff
  // Wait for confirmation message about fixed dGPS location before proceeding
  
  // Read minefield data (unordered list of mines) from base station, into linked list
  // TODO: ^
  // head of the linked list that stores the incoming mine data
  node_t* head = NULL;
  // gather incoming minefield data and place it into a linked list
  // This call will be blocking as data is received
  get_minefield_data(&head);
  
  // Plan the drone's path through the minefield
  plan_path(home_lat, home_lon, &head);
  // At this point, all mines are stored in a global 2D array (n x 6) called "mines".
  // The current position in this array is storred in a global index called "mines_index"
  // The current mine can be accessed as mines[mines_index / 6][mines_index % 6]
  // When mines_index % 6 == 5 (or zero, depends on how we implement it) it's time to return to the base station to get more sandbags
}

void loop() {
  // Main state machine - this handles the primary program flow
  switch(state) {
    case DISARMED:      // Disarmed, sitting on the ground
      // If we get the go signal from the base station:
      //   arm();
      //   state = ARMED;
      // Not 100% sure it's possible to arm like this - needs more testing
      break;
    case ARMED:         // Armed, ready to takeoff
      // Just chilling? Might not need this one
      break;
    case TAKEOFF:       // Actively taking off
      takeoff();
      // listen for ACK response, and wait until complete
      if(command_status == ACCEPTED) {
        command_status = COMPLETED; // Mark it as taken care of
        // takeoff complete, now we can move on.
        // TODO: [update state here]
      }
      break;
    case LAND:          // Actively landing
    
      break;
    case APPROACH:      // Approaching a mine

      break;
    case READY_TO_DROP: // Drone is over the mine, time to drop

      break;
    case DROP:          //dropping the payload

      break;
    case ESCAPE:        // Drone just dropped a payload, should now be running away
    // TODO: After each drop, check to see how many payloads are left. If there are none, return to base

      break;
    case ABORT:         // User clicks the abort button and the drone needs to return to base

      break;
    case RETURN_HOME:   // Drone returns back to base
    // probably will land after this command happens :) 
    
      break;
  }
  
  // Check for incoming messages from the base station
  // TODO: [Insert function call here]
  
  // Check for incoming messages from the Pixhawk
  mavlink_message_t msg_in;
  mavlink_status_t stat_in;
  if(receive_mavlink(&msg_in, &stat_in)) {
    switch(msg_in.msgid) {
      case MAVLINK_MSG_ID_COMMAND_ACK:
        set_globals(&msg_in, &stat_in);
        break;
    }
  }
}

// Reads incoming minefield data from the base station and stored it in the linked list pointed to by head_ref
// Note: this call is blocking - it will not return until all of the minefield data has been received
void get_minefield_data(node_t** head_ref) {
//  while(we've got mines) {
    // read some minefield data...
    struct mine_t* new_mine = (struct mine_t*)malloc(MINE_T_SIZE);  // allocate some memory for the new mine data
    new_mine->lat = NULL; // TODO: Put a latitude here
    new_mine->lon = NULL; // TODO: Put a longitude here
    new_mine->isDetonated = false;  // assume the mine is live
    LL_add(head_ref, new_mine);  // Add the new mine to the linked list
//  }
}
