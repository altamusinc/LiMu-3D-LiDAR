# Limu ToF Sensor
## Introduction
Limu Lidar Sensor
## List of Components/Tools
- Ethernet cable to connect sensor to PC (or router)
- Limu Lidar Sensor
- Linux PC
- AC/DC Adapter with 24V and 4A output
- (Optional) Router with stable network connection
## Prerequisite
- Install ROS in your Ubuntu environment, please refer to the instruction:
  https://wiki.ros.org/Installation/Ubuntu
## Quick Start
1. Power on sensor and connect it to PC using network cable
2. Once connected, go to network settings and under 'Wired' click the configuration button
![Wired connection settings](samples/limu_tof_ros/limu_wired.png)
4. Refer to above screenshot. Click on 'IPv4', then under 'IPv4 Method' click 'Manual'
	- Under 'Address', add a static IP address to PC in the form of ```10.10.31.XXX```, the last 3 digits cannot be exactly 180 since that is the sensor's static IP address
	- Under 'Netmask', add ```255.255.255.0```
	- Finally, click 'Apply' and wait for PC to be connected
5. Run ```ifconfig``` to verify the above settings
6. Run ```ping 10.10.31.180``` to ensure there is a connection to sensor
	- If connection fails, try restarting sensor before this step
7. In limu_tos_ros folder, to view sample, run:
```
catkin_make
```
```
sudo -s
```
```
./run.sh
```
## Sample Application
In the sample application, there is the point cloud, depth map, and RGB views
![Sample Application Screenshot](samples/limu_tof_ros/limu_sample.png)
## Changing Configurations
In sensor configuration, there any many settings that will affect the image produced. Refer to below screenshot
![Sensor Congiuration](samples/limu_tof_ros/limu_config.png)
- **lens_type** can be changed between 3 modes, WideField, StandardField, and NarrowField. Refer to specs documentation for more details
- **frequency_modulation** can be used to change the frequency modulation. It is suggested to use the default (6MHz).
- **channel** can be used to allow multiple sensors to be used in close proximity without interfering with each other. Sensors on different channels will not affect the infared light of another sensor if they happen to be in the same room and/or used next to each other
- **image_type** can be changed between Distance and DistanceAmplitude. If DistanceAmplitude is chosen, an amplitude map will also be generated, but this will not show up by default and must be selected as well
- The three **integration_time_tof** settings will change the time in which the sensor collects photons to create a signal. A smaller integration time will detect close and/or reflective (such as white coloured) objects, but not other objects. A larger integration time will detect close and/or reflective objects too indiscriminately, but will detect further and/or non-reflective (such as black coloured) objects. Depending on hdr_mode, different integration times out of the 3 given will be used
- **hdr_mode** will change which and how many integration times are used
	- In HDR_off, only the first integration time is used
	- In HDR_spatial, the only the first integration time is used, but it is split into 2, with one being the original integration time and the other being half of that. The two are then combined, giving the benefits of using two integration times at the cost of halving the image's resolution. This preserves the framerate, unlike HDR_temporal.
	- In HDR_temporal, all three are used; this can give the benefits of both small and large integration times, meaning objects both close and far, reflective and non-reflective can be detected. However, this may result in a lower framerate than other modes
- **min_amplitude** can be used to remove any noise that can affect the image by causing the image to become less sensitive to ambient infared light, such as from the sun. It can also be used to ignore further and/or les reflective objects in the image at higher values
## Developing you own application
## The basics
## Troubleshooting
- Ensure sensor is plugged in
- Ping sensor to ensure it is plugged in
- Remember to use a different static IP address than the sensor for your PC
## Contact
Contact us:  [info@vsemi.io](mailto:info@vsemi.io)
