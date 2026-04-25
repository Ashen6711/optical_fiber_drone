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
#include <c_library_v2-master/max/mavlink_msg_cam_trigger.h>
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

#define END_OF_VID 0x04
#define START_OF_VID 0x67

#define DEBUG 1

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

/**@todo: Bridge between FC and eth
**/
#define TX_ESP 1
#define RX_ESP 2
#define ORANGE_BAUD 57600

#define ORANGE_SERIAL Serial1

#define MAX_UDP_SIZE 1350 //1472 
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

#define PARSE_PAC_SEG 4

#define FRAME_HEADER_SIZE sizeof(frame_header)

WiFiUDP udp;

const int localPort = 14550; 
IPAddress pcIP(192, 168, 1, 50);
//IPAddress pcIP(192, 168, 0, 104); 
const int pcPort = 14550;

static bool eth_connected = false;
static bool cam_ok = false;

uint8_t system_id = 1;
uint8_t component_id = 1;

uint8_t target_sys_id = 1;
uint8_t target_comp_id = 1;

uint32_t heartbeat_timer = 0;

uint16_t frame_counter = 0;

uint8_t bufz[MAVLINK_MAX_PACKET_LEN]; //more than MAVLINK_MAX_PACKET_LEN

uint8_t cam_trigger = 0;

unsigned long parse_start_time;
bool parse_time_running = false;

bool default_params = false;
bool cam_update = true;

static const uint8_t crc8_table[256] = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
    0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
    0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
    0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
    0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
    0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
    0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
    0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
    0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
    0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
    0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
    0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
    0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
    0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
    0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
    0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
    0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

uint8_t send_not_ack = 0;
uint8_t to_parse_data[4];
uint8_t to_parse_data_[4];
int to_parse_data_len_ = 0;

uint8_t not_ack_tries = 3;

bool last_h_beat = false;
bool last_dist = false;
bool last_flow = false;

bool enable_cam = false;
bool enable_fc = false;
bool enable_joy = false;

struct __attribute__((packed)) frame_header {
  uint16_t frame_id;      // frame no.
  uint16_t fragment_id;   // fragment no. in frame_id
  uint16_t total_fragments; // total frags
  float voltage;
  //uint16_t data_len;      
};

struct __attribute__((packed)) params {
    uint8_t fc  : 1;
    uint8_t cam : 1; 
    uint8_t joy_control: 1;
    uint8_t padding1 : 5;

    uint8_t res_i : 4;
    uint8_t padding2: 4;

    uint8_t heartbeat: 1;
    uint8_t distance_sensor: 1;
    uint8_t optical_flow_rad: 1;
    uint8_t padding3: 5;
};

params paramsz;

const params default_p = {
    .fc = 0,               
    .cam = 1,              
    .joy_control = 0,      
    .padding1 = 0,

    .res_i = 0, //By default its VGA when cam_init           
    .padding2 = 0,

    .heartbeat = 0,        
    .distance_sensor = 0,  
    .optical_flow_rad = 0,
    .padding3 = 0
};

struct __attribute__((packed)) vid_reso {
    uint8_t debug : 1;
    uint8_t fc  : 1;
    uint8_t cam : 1; 
    uint8_t joy_control: 1;
};

enum Parse_State {
  WAITING,
  PARSING, 
  RETRY, 
  DONE 
  };
Parse_State parse_state = WAITING;

//CRC shit
uint8_t crc8_calculate(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00; 
    
    for (size_t i = 0; i < len; i++) {
        crc = crc8_table[data[i] ^ crc];
    }

    return crc;
}

void default_params_apply() {
    paramsz = default_p;
    enable_cam = paramsz.cam;
    enable_fc = paramsz.fc;
}

void handle_to_parse_packet(uint8_t *packet_buffer, size_t packet_len) {
    size_t payload_len = packet_len - 1; 
    uint8_t received_crc = packet_buffer[payload_len];
    uint8_t calc_crc = crc8_calculate(packet_buffer, payload_len);
    
    if (calc_crc == received_crc) {
        paramsz = *(params*)packet_buffer; 
        
        enable_cam = paramsz.cam;
        enable_fc = paramsz.fc;

      sensor_t *s = esp_camera_sensor_get();
      if (s)
      s->set_framesize(s, (framesize_t)paramsz.res_i);
          
        DEBUG_PRINTLN("Params updated");
        parse_state = DONE;
    } else {
        DEBUG_PRINTLN("CRC FAIL!");
        parse_state = RETRY;
    }
}


//@todo Check the below fucns
void request_mavlink_stream(uint32_t msg_id, int rate_hz) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    int32_t interval = (rate_hz <= 0) ? -1 : (1000000 / rate_hz);

    mavlink_msg_command_long_pack(
        system_id,    
        component_id, 
        &msg,
        target_sys_id,            
        target_comp_id,           
        MAV_CMD_SET_MESSAGE_INTERVAL, 
        0,            
        (float)msg_id,   
        (float)interval, 
        0, 0, 0, 0, 0
    );

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    ORANGE_SERIAL.write(buf, len);
}

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
    config.frame_size = FRAMESIZE_VGA;
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 20;
    config.fb_count = 2;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        DEBUG_PRINTF("Cam init failed with error 0x%x", err);
        return false;
    }
    
    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, 1);
    DEBUG_PRINTLN("Cam Start");

    return true;
}

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
      header->voltage = 6.7; //hardcoded for now
    else 
      header->voltage = 0;

    memcpy(buffer + FRAME_HEADER_SIZE, fb->buf + offset, frag_size);
    
    udp.beginPacket(pcIP, pcPort);
    udp.write(buffer, FRAME_HEADER_SIZE + frag_size);
    udp.endPacket();
    delayMicroseconds(100);
    
    offset += frag_size;
    fragment_id++;
    //delayMicroseconds(200);
  }
  
  DEBUG_PRINTF("Sent %u fragments\n", fragment_id);
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

void params_spam(){
  if (paramsz.heartbeat && !last_h_beat) {
    request_mavlink_stream(MAVLINK_MSG_ID_HEARTBEAT, 1); 
    last_h_beat = true;
  }

  if (paramsz.distance_sensor && !last_dist) {
    request_mavlink_stream(MAVLINK_MSG_ID_DISTANCE_SENSOR, 10);
    last_dist = true;
  }
  if (paramsz.optical_flow_rad && !last_flow) {
    request_mavlink_stream(MAVLINK_MSG_ID_OPTICAL_FLOW_RAD, 20);
    last_flow = true;
  }
}

void cam_sesh_control(){
  int pack_siz = udp.parsePacket();
  if (pack_siz > 0) {
    if(pack_siz == 1){
        uint8_t charzzz;
        udp.read(&charzzz, 1);

        if(charzzz == END_OF_VID){
        cam_trigger = 0;
        DEBUG_PRINTLN("Cam sesh ended");
        }

        if(charzzz == START_OF_VID){
        cam_trigger = 1;
        }
    }
    udp.flush();
  }
}

void setup() {
  if(DEBUG){
    Serial.begin(115200);
  }
  
  pinMode(CAM_ENABLE, OUTPUT);   
  digitalWrite(CAM_ENABLE, LOW);

  ORANGE_SERIAL.begin(ORANGE_BAUD, SERIAL_8N1, RX_ESP, TX_ESP);

  Network.onEvent(onEvent);
  
  ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_CS, ETH_PHY_IRQ, ETH_PHY_RST, 
            ETH_PHY_SPI_HOST, ETH_PHY_SPI_SCK, ETH_PHY_SPI_MISO, ETH_PHY_SPI_MOSI);
  
  while (!ETH.linkUp()) {
    DEUBUG_PRINT(".");
    delay(100);
  }
  
  ETH.config(
    IPAddress(192, 168, 1, 100),
    IPAddress(192, 168, 1, 50),
    IPAddress(255, 255, 255, 0)
  );

/**
  ETH.config(
    IPAddress(192, 168, 0, 200),      
    IPAddress(192, 168, 0, 1),        
    IPAddress(255, 255, 255, 0)
  );
**/
  
  udp.begin(localPort);

  cam_ok = init_cam();
  if (!cam_ok)
    DEBUG_PRINTLN("Cam Failure");
  else
    DEBUG_PRINTLN("Cam Success");

  parse_start_time = millis();
  //parse_time_running = true;
}

void loop() {
  if (!ETH.linkUp()){
    DEBUG_PRINTLN("Eth failed");
      return;
  }

  cam_sesh_control();

while (parse_state != DONE) {
    if (millis() - parse_start_time >= 10000) {
        default_params = true;
        parse_state = DONE;
        DEBUG_PRINTLN("Timeout, using default params");
        break; 
    }

    switch (parse_state) {
        case WAITING: {
            int pack_siz = udp.parsePacket();
            if (pack_siz == 4) {
                uint8_t buffzzz[4];
                udp.read(buffzzz, 4);
                memcpy(to_parse_data, buffzzz, 4);
                parse_state = PARSING;
            }
            break;
        }

        case PARSING:
            handle_to_parse_packet(to_parse_data, 4);
            break;

        case RETRY: {
            if (not_ack_tries >= 1) {
                --not_ack_tries;
                
                unsigned long retry_wait = millis();
                while(millis() - retry_wait < 5000) {
                    yield();
                }
                
                parse_state = WAITING;
            } else {
                parse_state = DONE;
                default_params = true;
            }
            break;
        }

        default:
            break;
    }

    yield(); 
}

  if(default_params){
    default_params_apply();
    default_params = false;
  }

  if(enable_cam && cam_trigger){
  uint32_t start = micros();

  camera_fb_t *fb = esp_camera_fb_get();
  uint32_t captured = micros();

  if (!fb) {
    DEBUG_PRINTLN("Capture failed");
    delay(200);
    return;
  }

  send_fraged_frame(fb);
  uint32_t sent = micros();  
      
  DEBUG_PRINTF("Capture: %lu us, Send: %lu us, Total: %lu us\n", captured - start, sent - captured, sent - start);
                    
  esp_camera_fb_return(fb);

  //send_fraged_frame(fb);
  //esp_camera_fb_return(fb);
  //delay(20); 
  }

  //Debug
  if(enable_fc){
  params_spam();

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
  }
}