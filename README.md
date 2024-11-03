# System configuration
## communication
master PC -(Wi-Fi) - (larm/rarm) USB-Serial converter - (UART) - kondoh7 - (ICS) - servo1 (pitch), servo3 (yaw)

# Software setup
1. Setup environment (first time) 
```
$ mkdir -p ~/ros/seisakuten_ws/src
$ cd ~/ros/seisakuten_ws
$ catkin init
$ cd ~/ros/seisakuten_ws/src
$ git clone https://github.com/nakane11/umoru_arm
$ catkin build

$ cd ~/ros/seisakuten_ws/src
$ git clone https://github.com/nakane11/rcb4.git -b umoru
$ cd rcb4
$ pip3 -e .
$ catkin b kxr_controller --cmake-args -DUSE_VIRTUALENV=OFF
```
2. Add udev rules
```
$ cd ~/ros/seisakuten_ws/src/umoru_arm
$ sudo ln -s 90-umoru.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules && udevadm trigger
```
4. Run arm controller  
```
$ rossetip
$ source ~/ros/seisakuten_ws/devel/setup.bash
$ roslaunch umoru_arm umoru_larm.launch
```
```
$ rossetip
$ source ~/ros/seisakuten_ws/devel/setup.bash
$ roslaunch umoru_arm umoru_rarm.launch
```

## B. Client (master PC)
1. Start `roscore`
2. Publish ros messages via MotionClient
```
import rospy
from umoru_arm import MotionClient

rospy.init_node("test")
client = MotionClient("both")

client.reset_pose()
client.init_pose()
```
# Angle range
## rarm
### servo 1 (pitch)
- -90 -> up
-  0   -> horizontal
-  90 -> down
### servo 3 (yaw)
- -135 -> extended
-  -50   -> bent inward

## larm
### servo 1 (pitch)
- 90 -> up
-  0   -> horizontal
-  -90 -> down
### servo 3 (yaw)
- 135 -> extended
-  50   -> bent inward
