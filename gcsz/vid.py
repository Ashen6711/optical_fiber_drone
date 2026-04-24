import socket
import struct
import io
from PIL import Image, ImageDraw, ImageFont
import cv2
import numpy as np
import time
import random
from pymavlink import mavutil
import time
import configparser
import os

PC_PORT = 14550
BOARD_PORT = 14550
BOARD_IP = "192.168.1.100"

FONT_PATH = r"/home/ashen/optical_fiber_drone/gcsz/uav_osd/UAV-OSD-Mono.ttf"
FONT = ImageFont.truetype(FONT_PATH, size=10)

HEADER_FMT  = '<HHHf'
HEADER_SIZE = struct.calcsize(HEADER_FMT)

END_OF_VID = b'\x04'
START_OF_VID = b'\x67'
refresh = True

INI_FILE = r'/home/ashen/optical_fiber_drone/gcsz/params.ini'
to_parse = True

frame_voltage = {}

fragments = {}
frame_stats = {}
wait = 0
frames_no = 0
frames_ = True

jitter_frame = 0

fps_timer = time.time()
fps = 0

'''
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

master.mav.cam_trigger_send(
    1,  
    1, 
    1                        
)
print("Camera triggered")
'''

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", PC_PORT))
sock.settimeout(1.0)

print(f"Listening for video on port {PC_PORT}")

def calculate_crc8(data):
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

def generate_payload(config_file):
    config = configparser.ConfigParser()
    config.read(config_file)

    byte1 = 0
    if config.getboolean('sym_params', 'enable_fc'):          
        byte1 |= (1 << 0)
    if config.getboolean('sym_params', 'enable_cam'):         
        byte1 |= (1 << 1)
    if config.getboolean('sym_params', 'enable_joy_control'): 
        byte1 |= (1 << 2)

    byte2 = config.getint('vid_reso', 'reso_i') & 0xFF

    byte3 = 0
    if config.getboolean('mavlink_tele', 'msg_heartbeat'):  
        byte3 |= (1 << 0)
    if config.getboolean('mavlink_tele', 'msg_distance_sensor'): 
        byte3 |= (1 << 1)
    if config.getboolean('mavlink_tele', 'msg_optical_flow_rad'):        
        byte3 |= (1 << 2)

    payload = bytearray([byte1, byte2, byte3])
    payload.append(calculate_crc8(payload))
    return payload


def draw_text(frame, text, pos, color, opacity):
    global jitter_frame
    jitter_frame += 1

    img_pil = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)).convert("RGBA")
    overlay = Image.new("RGBA", img_pil.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)

    x, y = pos
    r, g, b = color

    jx = random.randint(-1, 1) if jitter_frame % 10 == 0 else 0
    #jy = random.randint(-1, 1) if jitter_frame % 7 == 0 else 0

    #draw.text((x + jx - 2, y + jy), text, font=FONT, fill=(r, 0, 0, opacity))   
    #draw.text((x + jx, y + jy), text, font=FONT, fill=(0, g, 0, opacity))   
    #draw.text((x + jx + 2, y + jy), text, font=FONT, fill=(0, 0, b, opacity))   
    draw.text((x + jx, y), text, font=FONT, fill=(r,g,b, opacity))

    img_pil = Image.alpha_composite(img_pil, overlay)
    return cv2.cvtColor(np.array(img_pil.convert("RGB")), cv2.COLOR_RGB2BGR)

def process_frame(frame_id, fragments_list, voltage):
    try:
        data = b''.join([f[1] for f in sorted(fragments_list)])

        image = Image.open(io.BytesIO(data))
        frame = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)

        frame = draw_text(frame, f"{fps:.1f}FPS", pos=(10, 5), color=(255, 255, 255), opacity = 180)
        volt_text = f"{voltage:.2f}V"
        text_w = FONT.getlength(volt_text)
        cx = int((frame.shape[1] - text_w) / 2)
        frame = draw_text(frame, volt_text, pos=(cx, 5), color=(255, 255, 255), opacity=200)

        cv2.imshow("FPV", frame)
        cv2.waitKey(1)

        print(f"Frame {frame_id}: {len(data)} bytes, {image.size}")
        return True
    
    except Exception as e:
        print(f"Frame {frame_id} decode error: {e}")
        return False

while True:
    if INI_FILE.lower().endswith('.ini') and os.path.exists(INI_FILE) and to_parse:
        binary_data = generate_payload(INI_FILE)
        print(f"Payload: {binary_data.hex().upper()}")
        sock.sendto(binary_data, (BOARD_IP, PC_PORT))
        to_parse = False

    if(refresh):
        sock.sendto(START_OF_VID, (BOARD_IP, PC_PORT))
        refresh = False
    
    try:
        data, addr = sock.recvfrom(1600)
        #print(f"Packet from {addr}, size: {len(data)}") 

        if not data:
            continue

        if len(data) < 8:
            continue

        frame_id, fragment_id, total_fragments, voltage = struct.unpack(HEADER_FMT, data[:HEADER_SIZE])
        payload = data[HEADER_SIZE:]

        if fragment_id == 0:
            frame_voltage[frame_id] = voltage

        #print(f"Frame {frame_id}, frag {fragment_id}/{total_fragments}")

        if frame_id not in fragments:
            fragments[frame_id] = {}
            frame_stats[frame_id] = (total_fragments, set())

        fragments[frame_id][fragment_id] = payload
        frame_stats[frame_id][1].add(fragment_id)

        received = len(fragments[frame_id])
        if received == (total_fragments):
            frame_data = [(i, fragments[frame_id][i]) for i in range(total_fragments)]
            voltage = frame_voltage.get(frame_id, 0.0) 
            if(process_frame(frame_id, frame_data, voltage)):
                now = time.time()
                elapsed = 1/(now - fps_timer)
                fps = 0.9 * fps + (1-0.9) * elapsed
                fps_timer = now

            del fragments[frame_id]
            del frame_stats[frame_id]
            frame_voltage.pop(frame_id, None)
        
        frames_ = True
        wait = 0
        frames_no += 1

        old_frames = [fid for fid in fragments if fid < frame_id - 10]
        for fid in old_frames:
            del fragments[fid]
            if fid in frame_stats:
                del frame_stats[fid]

    except socket.timeout:
        if(frames_):
            print(frames_no)
            frames_ = False
        wait+=1
        print(wait)
        continue
    except KeyboardInterrupt:
        sock.sendto(END_OF_VID, (BOARD_IP, PC_PORT))
        break

cv2.destroyAllWindows()