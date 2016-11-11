# Setup the KUKA youBot onboard PC

## Software
|-|-|-|-|
|**Software**|**Name**|**Version**|**Remarks**|
|Operating System|Ubuntu|12.04 TLS||
|Meta Operating System|ROS|Hydro||
|ROS Nodes||||

## Hardware
- KUKA youBot base
  - omni-directional mobile platform
  - real-time EtherCAT communication
  - power supply
  - on-board PC, mini ITX PC-Board with embedded CPU, 2 GB RAM, 32 GB SSD Flash, USB
- KUKA youBot manipulator
  - 5-degree-of-freedom
  - 2-finger gripper
  - real-time EtherCAT communication
- arm and platform can be used independently
- SICK Tim551 Laser Scanner
  - Indoor and Outdoor usage
  - Aperture angle: 270 °
  - Operating range: 0.05 m ... 25 m
  - Scanning frequency: 15 Hz
  - Serial (via USB) and Ethernet Connection
- ASUS Xtion PRO Live Camera
  - Sensors: RGB, Depth, Microphone
  - Depth Image Size: VGA (640x480), 30FPS or QVGA(320x240), 60fps
  - Field of View: 58 ° H, 45 ° V,  70 ° D
  - Range: 0.8 m ... 3.5 m
  - USB-Connection

## 1. Install the preconfigured Ubuntu with ROS
As a first step you have to prepare a USB Stick (4 GB) as a bootable device. Therefore go to the [youBot-website](http://www.youbot-store.com/developers/remastered-ubuntu-linux) and dowload the ROS Hydro Image Version 1.0.1. Create your live-boot USB-Stick and plug it into one of the USB ports of the KUKA youBot onboard PC. Start the PC and follow the onscreen installation instruction. They are very similar to the Ubuntu installation steps.

**Hint:** If the PC boots into an already installed OS on the SSD, you have to change the Boot Order and Disk Order in the BIOS.

## 2. Setup WLAN Connection
We used a Fritz! WLAN-USB-Stick for connecting the youBot with the WLAN of the DHBW. It's a plug and play device, therefore we just plug it in and waited until the driver was installed automatically. After that we were able to connect to the WLAN. This allowed us to load and install new software with the `apt`-package manager and use a webbrowser to do same research.

## 3. Test installation and youBot driver
If you want to test your installation there are some sample application already installed from the live-boot image. You will need at least three terminals to run the following commands.
- Start the ROS master node:
``` bash
roscore
```
- Launch the youBot driver interface node:
``` bash
roslaunch youbot_driver_ros_interface youbot_driver.launch
```
- Teleoperate the youBot with the following node. You can use your keyboard to send commands to the youBot base.
``` bash
rosrun youbot_driver_ros_interface youbot_keyboard_teleop.py
```
Alternatively you can send a new joint position to the youBot manipulator arm with `rostopic`. This command will rotate the arm to a new angle (_1.552 rad = 90 degree_).

``` yml
rostopic pub -1 /arm_1/arm_controller/position_command brics_actuator/JointPositions "
poisonStamp:
 originator: ''
 description: ''
 qos: 0.0
positions:
- timeStamp:
   secs: 0
   nsecs: 0
 joint_uri: 'arm_joint_1'
 unit: 'rad'
 value: 2.0"
```

**Hint:** The youBot PC has got two seperate ethernet interfaces. You should check, which interface is used for the EtherCAT communication with the youBot motors and sensors. In our case we had to change the driver configuration to use the second ethernet interface called `eth1`. The configuration file can be found in `/opt/ros/hydro/share/youbot_driver/config/youbot_base.cfg`. You need root permissions to make changes to the file.

**Hint:** After you have installed the KUKA youBot driver, you have to give the driver raw acess to the ethernet interface where the motors and sensors are connected to. Run the following commands to do this:
``` bash
sudo setcap cap_net_raw+ep /opt/ros/hydro/lib/youbot_driver_ros_interface/youbot_driver_ros_interface
sudo ldconfig /opt/ros/hydro/lib
```
