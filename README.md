# System configuration
## communication
master PC -(Wi-Fi) - (larm/rarm) radxa - (UART) - kondoh7 - (ICS) - servo1 (pitch), servo3 (yaw)
## power supply
Lipo battery (2.9-12V) - pump_power_board (12V) - kondoh7 - servo1, 3

# Software setup
## radxa for each arm
1. `ssh USER@IP_ADRESS`
2. Setup environment  
```
$ mkdir -p ~/ros/seisakuten_ws/src
$ cd ~/ros/seisakuten_ws
$ catkin init
$ cd ~/ros/seisakuten_ws/src
$ git clone https://github.com/nakane11/umoru_arm
$ catkin build
```
4. Connect to master PC  
`rossetmaster MASTER_IP_ADDRESS`
5. Run arm controller  
```
$ source ~/ros/seisakuten_ws/devel/setup.bash
$ roslaunch umoru_arm umoru_arm.launch arm:="rarm"
```

## master PC
1. Start roscore
2. Publish ros messages via MotionClient
```
import rospy
from umoru_arm import MotionClient

rospy.init_node("test")
client = MotionClient("rarm")

client.reset_pose()
client.init_pose()
```
## Angle range
### servo 1 (pitch)
- -90 -> up
-  0   -> horizontal
-  90 -> down
### servo 3 (yaw)
- -135 -> extended
-  -40   -> bent inward
