Hardware required(this is what I am using):
1. ESP32-S3-ETH: https://www.waveshare.com/wiki/ESP32-S3-ETH
2. Single Mode Single Fiber SFP Module: https://www.amazon.in/Syrotech-1-25G-Single-Fiber-Module/dp/B083M1JKD4?source=ps-sl-shoppingads-lpcontext&ref_=fplfs&psc=1&smid=A164ZRJKKAHS32
3. SFP Media Converter: https://www.amazon.in/TP-Link-MC220L-Tp-Link-Fiber-Converter/dp/B001GQDRWK?th=1
4. OVA3660 (any such related cam)
5. FC: Orange cube plus 

Features:
1. FPV: Custom vid res streaming @ 27-28fps (with OSD-like overlay)
2. Joystick manual control over mavlink (want to explore parsing of rc channels override into crsf protocol)
3. FC and ETh dev board telemetry comms

@todo
1. For now the code is blocking, will be integrating FreeRTOS on esp32-s3 and asyncio for concurrency on GCS side
2. Custom osd map overlay
3. For now using simple .ini files, will make it cleaner ig

So, I came across this vid: https://www.youtube.com/watch?v=1QRrsfQfzxw and https://gitea.osmocom.org/electronics/osmo-small-hardware/src/branch/master/sfp-breakout/gerber/sfp-breakout

Might be implementing this if I manage to do it on a zero-pcb.

Examples/Demos: Coming soon ig!
Till then check wiring yay: https://drive.google.com/file/d/1Hoop3sKOpznAGxQl1uySULUkNfj7XI21/view?usp=sharing

Note: If I haven't pushed for a while, I am working on custom FC
