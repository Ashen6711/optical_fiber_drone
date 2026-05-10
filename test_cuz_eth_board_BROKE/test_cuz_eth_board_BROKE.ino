#include "/home/ashen/optical_fiber_drone/optical_fiberz/lib/c_library_v2-master/common/mavlink.h"

#define ORANGE_SERIAL Serial1
#define DEBUG 1

#if DEBUG
  #define DEBUG_PRINT(x, ...) Serial.print(x, ##__VA_ARGS__)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINTF(x, ...) Serial.printf(x, ##__VA_ARGS__)
#else
  #define DEBUG_PRINT(x, ...)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(x, ...)
#endif

#define LED 13

const uint8_t MY_SYS_ID = 250;
const uint8_t MY_COMP_ID = 167;
const uint8_t TARGET_SYS_ID = 1;
const uint8_t TARGET_COMP_ID = 1;

mavlink_message_t msg;
mavlink_status_t status;

const unsigned long req_delay = 2000;
bool req_sent = false;

unsigned long now = 0;

void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  Serial.begin(115200);
  ORANGE_SERIAL.begin(115200);
  
  now = millis();
}

void loop() {
  digitalWrite(LED, LOW); 

  if (!req_sent && now >= req_delay) {
    set_msg_intervalz(MAVLINK_MSG_ID_HEARTBEAT, 1000000);
    set_msg_intervalz(MAVLINK_MSG_ID_DISTANCE_SENSOR, 100000);
    req_sent = true;
  }

  reed_mav();
}

void set_msg_intervalz(uint16_t msg_id, int32_t interval_us) {
  mavlink_message_t cmd;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(MY_SYS_ID, MY_COMP_ID, &cmd, TARGET_SYS_ID, TARGET_COMP_ID, MAV_CMD_SET_MESSAGE_INTERVAL, 0, (float)msg_id, (float)interval_us, 0, 0, 0, 0, 0);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &cmd);
  ORANGE_SERIAL.write(buf, len);
}

void reed_mav() {
  while (ORANGE_SERIAL.available()) {
    uint8_t c = ORANGE_SERIAL.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      handle_msgs(&msg);
    }
  }
}

void handle_msgs(mavlink_message_t *msg) {
  switch (msg->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: {
      DEBUG_PRINT("Heart: ");
      mavlink_heartbeat_t hb;
      mavlink_msg_heartbeat_decode(msg, &hb);
      DEBUG_PRINTLN(hb.autopilot);
      break;
    }

    case MAVLINK_MSG_ID_DISTANCE_SENSOR: {
      DEBUG_PRINT("Distance: ");
      mavlink_distance_sensor_t dist;
      mavlink_msg_distance_sensor_decode(msg, &dist);
      DEBUG_PRINTLN(dist.current_distance);
      break;
    }

    default:
      break;
  }
}