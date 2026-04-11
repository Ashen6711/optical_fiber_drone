
/**Features:
1. FPV:
@todo: OSD on host GCS, hence send telemetry 
2. Manual control:
@todo
3. FC bridge:
@todo
**/
#include <ETH.h>
#include <WiFiUdp.h>
#include <c_library_v2-master/common/mavlink.h>
#include "esp_camera.h"

void handle_mavlink_message(mavlink_message_t *msg);
void onEvent(arduino_event_id_t event, arduino_event_info_t info);

//https://www.waveshare.com/wiki/ESP32-S3-ETH
#define ETH_PHY_TYPE     ETH_PHY_W5500
#define ETH_PHY_ADDR     1
#define ETH_PHY_CS       14   
#define ETH_PHY_IRQ      10  
#define ETH_PHY_RST      9    
#define ETH_PHY_SPI_HOST SPI2_HOST
#define ETH_PHY_SPI_SCK  13   
#define ETH_PHY_SPI_MISO 12   
#define ETH_PHY_SPI_MOSI 11   

#define DEBUG 1
#define HEARBEAT_TEST 1
#define CAM_TEST 0
#define FC_TEST 0

#if CAM_TEST
#define CAM_ENABLE     8
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  3
#define SIOD_GPIO_NUM  48
#define SIOC_GPIO_NUM  47
#define Y9_GPIO_NUM    18
#define Y8_GPIO_NUM    15
#define Y7_GPIO_NUM    38
#define Y6_GPIO_NUM    40
#define Y5_GPIO_NUM    42
#define Y4_GPIO_NUM    46
#define Y3_GPIO_NUM    45
#define Y2_GPIO_NUM    41
#define VSYNC_GPIO_NUM 1
#define HREF_GPIO_NUM  2
#define PCLK_GPIO_NUM  39
#endif

/**@todo: Bridge between FC and eth
**/
#if FC_TEST
#define TX_ESP 1
#define RX_ESP 2
#define ORANGE_SERIAL Serial1
#define ORANGE_BAUD 57600
#endif

#define MAX_UDP_SIZE 1200 //1472 
//#define FRAME_HEADER_SIZE sizeof(frame_header)

#if DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINTF(x, ...) Serial.printf(x, ##__VA_ARGS__)

  #define MAVLINK_HANDLE(msg) \
    do { \
      handle_mavlink_message(msg); \
    } while(0)

  /**
  #define ON_EVENT(event, info) \
    do { \
      onEvent(event, info); \
    } while(0)
    **/
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define MAVLINK_HANDLE(msg)
  #define DEBUG_PRINTF(x, ...)
  //#define ON_EVENT(event, info)
#endif

WiFiUDP udp;

const int localPort = 14550; 
//IPAddress pcIP(192, 168, 1, 50);
IPAddress pcIP(192, 168, 0, 104); 
const int pcPort = 14550;

static bool eth_connected = false;
static bool cam_ok = false;

uint8_t system_id = 1;
uint8_t component_id = 1;

/**
uint8_t target_sys_id = 1;
uint8_t target_comp_id = 1;
**/

uint32_t heartbeat_timer = 0;

uint16_t frame_counter = 0;

uint8_t bufz[MAVLINK_MAX_PACKET_LEN]; //more than MAVLINK_MAX_PACKET_LEN

struct __attribute__((packed)) frame_header {
  uint16_t frame_id;      // frame no.
  uint16_t fragment_id;   // fragment no. in frame_id
  uint16_t total_fragments; // total frags
  float voltage;
  //uint16_t data_len;      
};

#define FRAME_HEADER_SIZE sizeof(frame_header)

/**
@todo: Add more messages
**/
void handle_mavlink_message(mavlink_message_t *msg) {
  switch (msg->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
      DEBUG_PRINTLN("Heartbeat message received"); 
      break;
    default:
      DEBUG_PRINTF("Msg ID: %d\n", msg->msgid);
  }
}

/**
*@todo Check the below fucns
void request_stream(uint8_t stream_id, uint8_t rate_hz) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(
    1,
    200,
    &msg,
    11,    
    67,    
    MAV_CMD_SET_MESSAGE_INTERVAL, 
    0,    
    (float)stream_id,     
    (float)1000000.0f / rate_hz, 
    0, 0, 0, 0, 0        
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  ORANGE_SERIAL.write(buf, len);
}
**/

/**
}
void request_stream(uint8_t stream_id, uint8_t rate_hz) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_request_data_stream_pack(
    11, //@todo
    67, //@todo
    &msg,
    1,          
    1,          
    stream_id,
    rate_hz,
    1           
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  ORANGE_SERIAL.write(buf, len);
}
**/

void onEvent(arduino_event_id_t event, arduino_event_info_t info) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      DEBUG_PRINTLN("ETH Started");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED: 
      DEBUG_PRINTLN("ETH Connected"); 
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:    
      DEBUG_PRINT("ETH Ready: ");
      DEBUG_PRINTLN(ETH.localIP());
      eth_connected = true;
      break;
    case ARDUINO_EVENT_ETH_STOP:
      DEBUG_PRINTLN("Eth Stopped");
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      DEBUG_PRINTLN("Eth disconnected");
      eth_connected = false;
      break;
    default: break;
  }
}

/**
bool init_cam() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  
  config.xclk_freq_hz = 8000000;        
  config.pixel_format = PIXFORMAT_JPEG;  
  config.frame_size = FRAMESIZE_QVGA;     
  config.jpeg_quality = 12;              
  config.fb_count = 1;       
  config.grab_mode = CAMERA_GRAB_LATEST;        
  config.fb_location = CAMERA_FB_IN_PSRAM;    

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Cam init failed: 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID)
    Serial.println("OV3660 detected!");
  else
    Serial.println(s->id.PID);
  
  Serial.println("Cam initialized");
  return true;
}
**/

#if CAM_TEST
bool init_cam() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.frame_size = FRAMESIZE_QVGA; //FRAMESIZE_QQVGA
    config.pixel_format = PIXFORMAT_JPEG; // for streaming
    //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 20;
    config.fb_count = 1;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        DEBUG_PRINTF("Cam init failed with error 0x%x", err);
        return false;
    }

    DEBUG_PRINTLN("Cam Start");
    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, 1);

    return true;
}
#endif

/**todo: Learn about UDP fragmentation
**/
void send_fraged_frame(camera_fb_t *fb) {
  if (!fb || !fb->buf || fb->len == 0) return;
  
  uint16_t frame_id = frame_counter++;
  uint16_t total_frag = (fb->len + MAX_UDP_SIZE - 1) / MAX_UDP_SIZE;
  
  DEBUG_PRINTF("Frame %lu: %u bytes, %u fragments\n", frame_id, fb->len, total_frag);
  
  uint8_t buffer[MAX_UDP_SIZE + FRAME_HEADER_SIZE];
  uint16_t fragment_id = 0;
  size_t offset = 0;
  
  while (offset < fb->len) {
    size_t frag_size = min((size_t)MAX_UDP_SIZE, fb->len - offset);
    
    frame_header *header = (frame_header *)buffer;
    header->frame_id = frame_id;
    header->fragment_id = fragment_id;
    header->total_fragments = total_frag;

    if (fragment_id == 0) 
      header->voltage = 6.7;
    else 
      header->voltage = 0;

    memcpy(buffer + FRAME_HEADER_SIZE, fb->buf + offset, frag_size);
    
    udp.beginPacket(pcIP, pcPort);
    udp.write(buffer, FRAME_HEADER_SIZE + frag_size);
    udp.endPacket();
    //delayMicroseconds(200);
    
    offset += frag_size;
    fragment_id++;
    delayMicroseconds(200);
  }
  
  DEBUG_PRINTF("Sent %u fragments\n", fragment_id);
}

void send_heartbeat() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_heartbeat_pack(
    system_id, 
    component_id, 
    &msg,
    MAV_TYPE_GENERIC,           
    MAV_AUTOPILOT_GENERIC,      
    MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,
    0,                         
    MAV_STATE_ACTIVE);         
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  DEBUG_PRINTLN(len);
  
  udp.beginPacket(pcIP, pcPort);
  udp.write(buf, len);
  udp.endPacket();
  
  DEBUG_PRINTLN("Heartbeat is sent");
}

void check_for_mavlink() {
  int packetSize = udp.parsePacket();
  DEBUG_PRINTLN(packetSize);
  if (packetSize) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    int len = udp.read(buf, sizeof(buf));
    DEBUG_PRINTLN(len);
    
    mavlink_message_t msg;
    mavlink_status_t status;
    
    for (int i = 0; i < len; i++) {
      if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
        MAVLINK_HANDLE(&msg);
      }
    }
  }
}

void setup() {
  if(DEBUG){
    Serial.begin(115200);
  }
  
  #if CAM_TEST
  pinMode(CAM_ENABLE, OUTPUT);   
  digitalWrite(CAM_ENABLE, LOW);
  #endif

  #if FC_TEST
    ORANGE_SERIAL.begin(ORANGE_BAUD, SERIAL_8N1, RX_ESP, TX_ESP);
  #endif

  Network.onEvent(onEvent);
  
  ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_CS, ETH_PHY_IRQ, ETH_PHY_RST, 
            ETH_PHY_SPI_HOST, ETH_PHY_SPI_SCK, ETH_PHY_SPI_MISO, ETH_PHY_SPI_MOSI);
  
  while (!ETH.linkUp()) {
    Serial.print(".");
    delay(100);
  }
  
  /**
  ETH.config(
    IPAddress(192, 168, 1, 100),
    IPAddress(192, 168, 1, 50),
    IPAddress(255, 255, 255, 0)
  );
  **/

  ETH.config(
    IPAddress(192, 168, 0, 200),      
    IPAddress(192, 168, 0, 1),        
    IPAddress(255, 255, 255, 0)
  );
  
  udp.begin(localPort);

  #if CAM_TEST
  cam_ok = init_cam();
  if (!cam_ok)
    DEBUG_PRINTLN("Cam failure");
  else
    DEBUG_PRINTLN("Cam success");
  #endif
  //DEBUG_PRINTLN("MAVLink UDP ready on port 14550");

  //request_stream(DISTANCE_SENSOR, 1);
  //DEBUG_PRINTLN("Requested");
}

void loop() {
  if (!ETH.linkUp()){
    DEBUG_PRINTLN("Eth failed");
      return;
    }

  /**@todo: Add host GCS control
  **/
  if(CAM_TEST){
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    DEBUG_PRINTLN("Capture failed");
    delay(200);
    return;
  }

  send_fraged_frame(fb);
  esp_camera_fb_return(fb);
  delay(100); 
  }

  if(HEARBEAT_TEST){
  check_for_mavlink();
  
  if (millis() - heartbeat_timer > 1000) {
    heartbeat_timer = millis();
    send_heartbeat();
  }
  }

  //Debug
  #if FC_TEST
  mavlink_message_t msg;
  mavlink_status_t status;

  while(ORANGE_SERIAL.available()){
    uint8_t readz = ORANGE_SERIAL.read();
    if(mavlink_parse_char(MAVLINK_COMM_0, readz, &msg, &status)){
      DEBUG_PRINTLN("Read");
      uint16_t len = mavlink_msg_to_send_buffer(bufz, &msg);
      udp.beginPacket(pcIP, pcPort);
      udp.write(bufz, len);
      udp.endPacket();
      DEBUG_PRINTLN("Forwarded");
    }
  }
  #endif
}