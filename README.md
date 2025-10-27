# IBUS_Telemetry_AVR_Soft_1.1
This is simple combo sensor for Flysky ADHDS2A RC systems.
Based on Arduino MiniPro 5V 16MHz , NEO6M-7M-GPS and BME280 5V modules.
It communicates with Flysky receivers i.e FS-IA6b via Sebsor IBUS port.
Prototype has been built on doublesided prototype PCB.
Elements are mounted on both side, to assure compact sensor sizes.
I attach also schematics, PCB design (Eagle / Fusion), and cabinet stls for 3D print and Gerber files to make easy order for PCB's
For more modern GPS module i.e 8M PCB require minor patching - modules have different pinout.
Major issue is, that IBUS libraries i found and used for this project, do not support block send of GPS data, so popular FX-i6X transmitter doesn't display GPS data correctly.
I am using Radiomaster TX16S 4in1 with EbgeTX 2.11.3 open source firmware and it works well. So I attach GpsInfo widget for tis system.
TO DO - electronics has additional servo port to connect to free servo channel or IBUS servo port to update Home position and altitude. It is not implement in current firmware version.
Yhe easiest way is to use servo channel, but also possible to decode IBUS servo port signal to extract servo channel which is even not covered by receiver's servo terminal.
Feel free to use, fork and modify.
