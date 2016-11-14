# Setup the KUKA youBot onboard PC

## Software
|**Software**|**Name**|**Version**|**Remarks**|
|-|-|-|-|
|Operating System|Ubuntu|12.04 TLS|Delivered with the youBot-live-USB-Stick |
|Meta Operating System|ROS|Hydro|Delivered with the youBot-live-USB-Stick |
|ROS Nodes| <ul><li>youbot_driver</li> <li>youbot_driver_ros_interface</li> <li>youbot_navigation</li> <li>youbot_simulation</li> <li>youbot_applications</li> <li>youbot_ros_samples</li> <li>youbot_description</li> <li>sick_tim</li> <li>openni_camera</li> <li>openni_launch</li></ul>|ROS Hydro|Some delivered with the youBot-live-USB-Stick and some installed manually.|

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

``` bash
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

## 4. Setup SICK Tim551 laser Scanner
The SICK Tim551 laser scanner should be configured to have a link-local address on his ethernet interface. If you have to change the interface properties, please refer to the configuration steps on the [SICK Tim ROS wiki page](http://wiki.ros.org/sick_tim). The configuration program can only be run on a Windows machine with the SICK Tim laser scanner connected via USB. We used the IP: `169.254.8.76`.

After you have successfully configured the laser scanner, connect it to the youBot via ethernet. After that install the SICK Tim driver and ROS node via:
``` bash
sudo apt-get install ros-hydro-sick-tim
```
Now change the IP address in the launch file `/opt/ros/hydro/share/sick_tim/launch/sick_tim551*.launch` to `169.254.8.76`. The port is `2111`. You need root permissions for this operation as well.

If you are not able to ping the laser scanner under its IP address (`ping 169.254.8.76`) you have to change the Ubuntu ethernet interface configuration as well. Therefore configure the interface `eth0` (or `eth1` if eth0 is your interface for the youBot motors and sensors) to only allow link-local addresses. We used the graphical user interface to change the behaviour of `Wired Connection 1`. Select in the dropdown box of the tab IPv4 _l
Linklocal_ and save your changes.

**Hint:** We had to give the administrator (root user) a new password to be able to change the internet settings.

Start the ROS master node and test your laser scanner with:
``` bash
roslaunch sick_tim sick_tim551*.launch
```
Use the auto-completion feature to find the right name of the launch file. If there were no errors, you should now be able to display the sensor data in rviz.

## Setup ASUS Xtion PRO live camera
At first connect the camera to the youBot via USB. After that install the openni-driver as described in the [ROS wiki](http://wiki.ros.org/openni_camera).
``` bash
sudo apt-get install ros-hydro-openni-camera
sudo apt-get install ros-hydro-openni-launch
```
Now you are able to test the camera. Therefore be sure that the ROS master is running and type the following:
``` bash
roslaunch openni_launch openni.launch
```
In another terminal type either `rosrun image_view disparity_view image:=/camera/depth/disparity` for diaplaying the depth sensor data or `rosrun image_view image_view image:=/camera/rgb/image_color` to display the RGB images.
