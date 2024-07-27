# System configuration
## communication
master PC -(Wi-Fi) - (larm/rarm) radxa - (UART) - kondoh7 - (ICS) - servo1 (pitch), servo3 (yaw)
## power supply
Lipo battery (7.4V) - kondoh7 - servo1, 3

# Software setup
## A. Arm controller
### option 1: radxa for each arm
1. from pc, `ssh USER@IP_ADRESS`
2. Setup environment (first time) 
```
$ mkdir -p ~/ros/seisakuten_ws/src
$ cd ~/ros/seisakuten_ws
$ catkin init
$ cd ~/ros/seisakuten_ws/src
$ git clone https://github.com/nakane11/umoru_arm
$ catkin build
```
3. Connect to master PC  
`rossetmaster MASTER_IP_ADDRESS`
4. Run arm controller  
```
$ source ~/ros/seisakuten_ws/devel/setup.bash
$ roslaunch umoru_arm umoru_arm.launch arm:="rarm"
```
### option 2: master PC
1. Setup environment (first time) 
```
$ mkdir -p ~/ros/seisakuten_ws/src
$ cd ~/ros/seisakuten_ws
$ catkin init
$ cd ~/ros/seisakuten_ws/src
$ git clone https://github.com/nakane11/umoru_arm
$ catkin build
```
2. Run arm controller for each arm
```
$ source ~/ros/seisakuten_ws/devel/setup.bash
$ roslaunch umoru_arm umoru_arm.launch arm:="rarm" device:=/dev/ttyUSB0
```
```
$ source ~/ros/seisakuten_ws/devel/setup.bash
$ roslaunch umoru_arm umoru_arm.launch arm:="larm" device:=/dev/ttyUSB1
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
