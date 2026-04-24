#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>
#include <linux/uinput.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <errno.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <time.h>
#include <dirent.h>
#include <stdint.h>
#include "optical_fiberz/c_library_v2-master/common/mavlink.h" 

#define BTN_MAX 10

//@todo: Adjust accordingly
const uint8_t system_id = 1;
const uint8_t component_id = 1;
const uint8_t target_system_id = 1;


int16_t mav_x = 0;
int16_t mav_y = 0;
int16_t mav_z = 0;
int16_t mav_r = 0;
uint16_t mav_buttons = 0;

struct {
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t r;
    uint16_t buttons;   
} mav_manual_control_mapping;

struct mav_manual_control_mapping mav_manual_control_mappingz;

int main(int argc, char* argv[])
{
    // Open UDP socket
    const int socket_fd = socket(PF_INET, SOCK_DGRAM, 0);

    if (socket_fd < 0) {
        printf("socket error: %s\n", strerror(errno));
        return -1;
    }

    // Bind to port
    struct sockaddr_in addr = {};
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    inet_pton(AF_INET, "0.0.0.0", &(addr.sin_addr)); // listen on all network interfaces
    addr.sin_port = htons(14550); // default port on the ground

    if (bind(socket_fd, (struct sockaddr*)(&addr), sizeof(addr)) != 0) {
        printf("bind error: %s\n", strerror(errno));
        return -2;
    }

    // We set a timeout at 100ms to prevent being stuck in recvfrom for too
    // long and missing our chance to send some stuff.
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    if (setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        printf("setsockopt error: %s\n", strerror(errno));
        return -3;
    }

    struct sockaddr_in src_addr = {};
    socklen_t src_addr_len = sizeof(src_addr);
    bool src_addr_set = false;

    /**
    while (true) {
        // For illustration purposes we don't bother with threads or async here
        // and just interleave receiving and sending.
        // This only works  if receive_some returns every now and then.
        receive_some(socket_fd, &src_addr, &src_addr_len, &src_addr_set);

        if (src_addr_set) {
            send_some(socket_fd, &src_addr, src_addr_len);
        }
    }

    return 0;
    **/
}

void send_joy(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    mavlink_message_t message;

    mavlink_msg_manual_control_pack(
    system_id,                 
    component_id,            
    &message,
    target_system_id,       
    mav_manual_control_mapping.x, 
    mav_manual_control_mapping.y,   
    mav_manual_control_mapping.z,   
    mav_manual_control_mapping.r,
    mav_manual_control_mapping.buttons, 
    0,     
    0,      
    0, 0,  
    0, 0, 0, 0, 0, 0
);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const int len = mavlink_msg_to_send_buffer(buffer, &message);

    int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr*)src_addr, src_addr_len);
    if (ret != len) {
        printf("sendto error: %s\n", strerror(errno));
    } else {
        printf("Sent Joy\n");
    }
}

int to_mav(int value, int code) {
    if(code != ABS_Y)
    return ((value - 1024) * 1000) / 1024;
    else 
    return (value * 1000) / 2048;
}

int mapz(int code, int value) {
    if (code == ABS_THROTTLE) {
        if (value >= 0 && value <= 1022) return BTN_0; 
        if (value >= 1023 && value <= 1024) return BTN_1;
        if (value >= 1025 && value <= 2048) return BTN_2;
    }
    if (code == ABS_RY) {
        if (value >= 0 && value <= 1023) return BTN_3;
        if (value >= 1024 && value <= 2048) return BTN_4;
    }
    if (code == ABS_RZ) {
        if (value >= 0 && value <= 1022) return BTN_5; 
        if (value >= 1023 && value <= 1024) return BTN_6;
        if (value >= 1025 && value <= 2048) return BTN_7;
    }    
    if (code == ABS_RUDDER) {
        if (value >= 0 && value <= 1023) return BTN_8;
        if (value >= 1024 && value <= 2048) return BTN_9;
    }
    return -1;
}

void emit(int fd, int type, int code, int val){
    struct input_event ie;
    memset(&ie, 0, sizeof(ie));
    ie.type = type;
    ie.code = code;
    ie.value = val;
    write(fd, &ie, sizeof(ie));
}

int main(){
    const char *input_dev = "/dev/input/by-id/usb-OpenTX_Radiomaster_Pocket_Joystick_00000000001B-event-joystick";
    int in_fd = open(input_dev, O_RDONLY);
    if (in_fd < 0) return 1;

    int out_fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
    if (out_fd < 0) {
        close(in_fd);
        return 1;
    }

    ioctl(out_fd, UI_SET_EVBIT, EV_KEY);
    ioctl(out_fd, UI_SET_EVBIT, EV_ABS);

    ioctl(out_fd, UI_SET_KEYBIT, BTN_0);
    ioctl(out_fd, UI_SET_KEYBIT, BTN_1);
    ioctl(out_fd, UI_SET_KEYBIT, BTN_2);
    ioctl(out_fd, UI_SET_KEYBIT, BTN_3);
    ioctl(out_fd, UI_SET_KEYBIT, BTN_4);
    ioctl(out_fd, UI_SET_KEYBIT, BTN_5);
    ioctl(out_fd, UI_SET_KEYBIT, BTN_6);
    ioctl(out_fd, UI_SET_KEYBIT, BTN_7);
    ioctl(out_fd, UI_SET_KEYBIT, BTN_8);
    ioctl(out_fd, UI_SET_KEYBIT, BTN_9);

    ioctl(out_fd, UI_SET_ABSBIT, ABS_X);
    ioctl(out_fd, UI_SET_ABSBIT, ABS_Y);
    ioctl(out_fd, UI_SET_ABSBIT, ABS_RX);
    ioctl(out_fd, UI_SET_ABSBIT, ABS_Z);

    struct uinput_user_dev uidev;
    memset(&uidev, 0, sizeof(uidev));
    snprintf(uidev.name, UINPUT_MAX_NAME_SIZE, "Virtual Pocket Joystick");
    uidev.id.bustype = BUS_USB;
    uidev.id.vendor  = 0x1234;
    uidev.id.product = 0x5678;
    uidev.id.version = 1;

    uidev.absmin[ABS_X] = 0;
    uidev.absmax[ABS_X] = 2048;
    uidev.absmin[ABS_Y] = 0;
    uidev.absmax[ABS_Y] = 2048;
    uidev.absmin[ABS_RX] = 0;
    uidev.absmax[ABS_RX] = 2048;
    uidev.absmin[ABS_Z] = 0;
    uidev.absmax[ABS_Z] = 2048;

    if (write(out_fd, &uidev, sizeof(uidev)) < 0){
        close(in_fd);
        close(out_fd);
        return 1;
    }

    if (ioctl(out_fd, UI_DEV_CREATE) < 0){
        close(in_fd);
        close(out_fd);
        return 1;
    }

    sleep(1);

    int last_btn_state[10] = {0};

    int default_buttons[] = {BTN_0, BTN_3, BTN_5, BTN_8};
    int abs_thr_buttons[] = {BTN_0, BTN_1, BTN_2};
    int abs_ry_buttons[] = {BTN_3, BTN_4};
    int abs_rz_buttons[] = {BTN_5, BTN_6, BTN_7};
    int abs_rudder_buttons[] = {BTN_8, BTN_9};

    /**
    emit(out_fd, EV_ABS, ABS_X, 1024);
    emit(out_fd, EV_ABS, ABS_Y, 1024);
    emit(out_fd, EV_ABS, ABS_RX, 1024);
    emit(out_fd, EV_ABS, ABS_Z, 1024);
    **/

    for (int i = 0; i < sizeof(default_buttons)/sizeof(default_buttons[0]); i++) {
        emit(out_fd, EV_KEY, default_buttons[i], 1);
        last_btn_state[default_buttons[i]] = 1;
    }
    emit(out_fd, EV_SYN, SYN_REPORT, 0);

    /**@todo: Convert to mavlink supported
    **/
    struct input_event ev;
    while (read(in_fd, &ev, sizeof(ev)) > 0){
        if (ev.type == EV_ABS){
            if (ev.code == ABS_X || ev.code == ABS_RX || ev.code == ABS_Z) {
                //emit(out_fd, EV_ABS, ev.code, ev.value);
            }       
            else if (ev.code == ABS_Y){
                emit(out_fd, EV_ABS, ev.code, 2048-ev.value);
            }
            else if (ev.code == ABS_RUDDER) {
                int btn = mapz(ev.code, ev.value);
                for (int i = 0; i < 2; i++) {
                    int value = (abs_rudder_buttons[i] == btn) ? 1 : 0;
                    if (value != last_btn_state[abs_rudder_buttons[i]]) {
                        emit(out_fd, EV_KEY, abs_rudder_buttons[i], value);
                        last_btn_state[abs_rudder_buttons[i]] = value;
                    }
                }
            }
            else if (ev.code == ABS_RY) {
                int btn = mapz(ev.code, ev.value);
                for (int i = 0; i < 2; i++) {
                    int value = (abs_ry_buttons[i] == btn) ? 1 : 0;
                    if (value != last_btn_state[abs_ry_buttons[i]]) {
                        emit(out_fd, EV_KEY, abs_ry_buttons[i], value);
                        last_btn_state[abs_ry_buttons[i]] = value;
                    }
                }
            }
            else if (ev.code == ABS_RZ) {
                int btn = mapz(ev.code, ev.value);
                for (int i = 0; i < 3; i++) {
                    int value = (abs_rz_buttons[i] == btn) ? 1 : 0;
                    if (value != last_btn_state[abs_rz_buttons[i]]) {
                        emit(out_fd, EV_KEY, abs_rz_buttons[i], value);
                        last_btn_state[abs_rz_buttons[i]] = value;
                    }
                }
            }
            else if (ev.code == ABS_THROTTLE) {
                int btn = mapz(ev.code, ev.value);
                for (int i = 0; i < 3; i++) {
                    int value = (abs_thr_buttons[i] == btn) ? 1 : 0;
                    if (value != last_btn_state[abs_thr_buttons[i]]) {
                        emit(out_fd, EV_KEY, abs_thr_buttons[i], value);
                        last_btn_state[abs_thr_buttons[i]] = value;
                    }
                }
            }
        }
        emit(out_fd, EV_SYN, SYN_REPORT, 0);
    }

    ioctl(out_fd, UI_DEV_DESTROY);
    close(in_fd);
    close(out_fd);

    return 0;
}