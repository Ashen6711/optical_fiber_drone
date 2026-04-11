import socket
import struct
import io
from PIL import Image
import cv2
import numpy as np
import time

PC_PORT = 14550

fragments = {}
frame_stats = {}
wait = 0
frames_no = 0
frames_ = True

fps_timer = time.time()
fps = 9.4

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", PC_PORT))
sock.settimeout(1.0)

print(f"Listening for video on port {PC_PORT}")

def process_frame(frame_id, fragments_list):
    try:
        data = b''.join([f[1] for f in sorted(fragments_list)])

        image = Image.open(io.BytesIO(data))
        frame = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)

        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.imshow("ESP32-CAM", frame)
        cv2.waitKey(1)

        print(f"Frame {frame_id}: {len(data)} bytes, {image.size}")
        return True
    
    except Exception as e:
        print(f"Frame {frame_id} decode error: {e}")
        return False

while True:
    try:
        data, addr = sock.recvfrom(1600)
        #print(f"Packet from {addr}, size: {len(data)}") 

        if len(data) < 8:
            continue

        header = struct.unpack('<HHHH', data[:8])
        frame_id        = header[0]
        fragment_id     = header[1]
        total_fragments = header[2]
        payload = data[8:]

        #print(f"Frame {frame_id}, frag {fragment_id}/{total_fragments}")

        if frame_id not in fragments:
            fragments[frame_id] = {}
            frame_stats[frame_id] = (total_fragments, set())

        fragments[frame_id][fragment_id] = payload
        frame_stats[frame_id][1].add(fragment_id)

        received = len(fragments[frame_id])
        if received == (total_fragments):
            frame_data = [(i, fragments[frame_id][i]) for i in range(total_fragments)]
            if(process_frame(frame_id, frame_data)):
                now = time.time()
                elapsed = 1/(now - fps_timer)
                fps = 0.9 * fps + (1-0.9) * elapsed
                fps_timer = now

            del fragments[frame_id]
            del frame_stats[frame_id]
        
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
        break

cv2.destroyAllWindows()