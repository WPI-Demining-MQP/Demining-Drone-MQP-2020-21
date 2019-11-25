#include "mavlink.h"
#define SYS_ID 255
#define COMP_ID 1
#define TARGET_ID 1

mavlink_message_t msg;
mavlink_status_t stat;

void setup() {
  Serial.begin(115200);
  Serial1.begin(19200);
  Serial.println("ready");
  while(millis() < 3000) { check_msg(); }

  mavlink_message_t msg;
  mavlink_msg_command_long_pack(SYS_ID, COMP_ID, &msg, TARGET_ID, 0, MAV_CMD_COMPONENT_ARM_DISARM, 0, 1,0,0,0,0,0,0); // ARM

  uint8_t buf[100];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf,len);

  while(millis() < 5000) { check_msg(); }

  mavlink_msg_command_long_pack(SYS_ID, COMP_ID, &msg, TARGET_ID, 0, MAV_CMD_COMPONENT_ARM_DISARM, 0, 0,0,0,0,0,0,0); // DISARM
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf,len);
  while(1) { check_msg(); }
}

void loop() {
  check_msg();
}

void check_msg() {
  if(Serial1.available()) {
    uint8_t byte_in = Serial1.read();
//    Serial.print(millis());
//    Serial.print("\t");
//    Serial.println(byte_in, HEX);
    if(mavlink_parse_char(0, byte_in, &msg, &stat)) {
      
      if(msg.msgid == 0) {
        Serial.println("HEARTBEAT");
      }
      else if(msg.msgid == 77) {
        Serial.print("ACK of MSG_ID=");
        Serial.print(mavlink_msg_command_ack_get_command(&msg));
        Serial.print(" STATUS=");
        Serial.print(mavlink_msg_command_ack_get_result(&msg));
        Serial.print(" PROG=");
        Serial.println(mavlink_msg_command_ack_get_progress(&msg));
      }
      else if(msg.msgid == 22) {
        Serial.print("PARAM_VALUE ");
        char param[17];
        mavlink_msg_param_value_get_param_id(&msg, param);
        Serial.print(param);
        Serial.print(" VALUE=");
        Serial.println(mavlink_msg_param_value_get_param_value(&msg));
      }
      else {
        Serial.print("MSG_ID=");
        Serial.print(msg.msgid);
        Serial.print("\tSEQ=");
        Serial.print(msg.seq);
        Serial.print("\tCOMP=");
        Serial.print(msg.compid);
        Serial.print("\tSYS_ID=");
        Serial.println(msg.sysid);
      }
    }
  }
}
