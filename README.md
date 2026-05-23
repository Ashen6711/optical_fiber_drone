Hardware required(this is what I am using):
1. [ESP32-S3-ETH](https://www.waveshare.com/wiki/ESP32-S3-ETH) (moving to P4)
2. [Single Mode Single Fiber SFP Module](https://www.amazon.in/Hanutech-Module-Transceiver-Supporting-Single/dp/B09HXTXXMP/ref=asc_df_B09HXTXXMP?mcid=f2ee12abc2bc30f09d5f220162c64860&tag=googleshopdes-21&linkCode=df0&hvadid=709855510224&hvpos=&hvnetw=g&hvrand=17623882472216962793&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9062072&hvtargid=pla-1720669486926&psc=1&hvocijid=17623882472216962793-B09HXTXXMP-&hvexpln=0&gad_source=1)
3. [SFP Media Converter](https://www.amazon.in/TP-Link-MC220L-Tp-Link-Fiber-Converter/dp/B001GQDRWK?th=1)
4. OVA3660 (any such related cam)
5. FC: Orange cube plus and Pixhawk 2.4.8

Features:
1. FPV: Custom vid res streaming @ tbd fps (with OSD-like overlay)
2. Joystick manual control over mavlink (want to explore parsing of rc channels override into crsf protocol)
3. FC and ETh dev board telemetry comms

@todo
1. For now the code is blocking, will be integrating FreeRTOS on esp32-s3 and asyncio for concurrency on GCS side (almost done, need to review code and study)
2. Custom osd map overlay 
3. For now using simple .ini files, will make it cleaner ig

So, I came across this [vid](https://www.youtube.com/watch?v=1QRrsfQfzxw) and [this](https://gitea.osmocom.org/electronics/osmo-small-hardware/src/branch/master/sfp-breakout/gerber/sfp-breakout)

Examples/Demos: Coming soon ig!
Till then check [wiring](https://drive.google.com/file/d/1Hoop3sKOpznAGxQl1uySULUkNfj7XI21/view?usp=sharing) yay (will be takin pics of new setup soon)

Note: If I haven't pushed for a while, I am working on custom FC
