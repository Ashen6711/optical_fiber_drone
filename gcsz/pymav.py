from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

print("MAVLink waiting for heartbeat from ESP32...")

master.wait_heartbeat()
print(f"Heartbeat from system {master.target_system}, component {master.target_component}")

counter = 0

while True:
    msg = master.recv_match(blocking=False)
    if msg:
        print(f"[RX] {msg.get_type()}: {msg.to_dict()}")
    
    counter += 1
    if counter % 5 == 0:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,  
            33,
            0, 0, 0, 0, 0, 0
        )
        print("[TX] Command sent")
    
    time.sleep(1)