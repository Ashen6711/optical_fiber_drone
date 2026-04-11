#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>
#include <linux/uinput.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <dirent.h>

#define BTN_MAX 10

int mapz(int code, int value) {
    if (code == ABS_RY) {
        if (value >= 0 && value <= 1023) return BTN_0;
        if (value >= 1024 && value <= 2048) return BTN_1;
    }
    if (code == ABS_THROTTLE) {
        if (value >= 0 && value <= 1022) return BTN_2; 
        if (value >= 1023 && value <= 1024) return BTN_3;
        if (value >= 1025 && value <= 2048) return BTN_4;
    }    
    if (code == ABS_RZ) {
        if (value >= 0 && value <= 1023) return BTN_5;
        if (value >= 1024 && value <= 2048) return BTN_6;
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
    const char *input_dev = "/dev/input/by-id/usb-OpenTX_Radiomaster_Zorro_Joystick_00000000001B-event-joystick";
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
    snprintf(uidev.name, UINPUT_MAX_NAME_SIZE, "Virtual Zorro Joystick");
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
    int default_buttons[] = {BTN_0, BTN_2, BTN_5};
    int abs_thr_buttons[] = {BTN_2, BTN_3, BTN_4};
    int abs_ry_buttons[] = {BTN_0, BTN_1};
    int abs_rz_buttons[] = {BTN_5, BTN_6};

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

    struct input_event ev;
    while (read(in_fd, &ev, sizeof(ev)) > 0){
        if (ev.type == EV_ABS){
            if (ev.code == ABS_X || ev.code == ABS_Y || ev.code == ABS_RX) {
                emit(out_fd, EV_ABS, ev.code, ev.value);
            }
            else if (ev.code == ABS_Z) {
                emit(out_fd, EV_ABS, ev.code, 2048-ev.value);
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
                for (int i = 0; i < 2; i++) {
                    int value = (abs_rz_buttons[i] == btn) ? 1 : 0;
                    if (value != last_btn_state[abs_rz_buttons[i]]) {
                        emit(out_fd, EV_KEY, abs_rz_buttons[i], value);
                        last_btn_state[abs_rz_buttons[i]] = value;
                    }
                }
            }
        } else if (ev.type == EV_KEY){
            if (ev.code == BTN_SOUTH){
                emit(out_fd, EV_KEY, BTN_7, ev.value);
            }
            else if (ev.code == BTN_WEST){
                emit(out_fd, EV_KEY, BTN_8, ev.value);
            }
            else if (ev.code == BTN_Z){
                emit(out_fd, EV_KEY, BTN_9, ev.value);
            }
        }
        emit(out_fd, EV_SYN, SYN_REPORT, 0);
    }

    ioctl(out_fd, UI_DEV_DESTROY);
    close(in_fd);
    close(out_fd);

    return 0;
}