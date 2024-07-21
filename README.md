# System configuration
## communication
master PC -(Wi-Fi) - (larm/rarm) radxa - (UART) - kondoh7 - (ICS) - servo1 (pitch), servo3 (yaw)
## power supply
Lipo battery (2.9-12V) - pump_power_board (12V) - kondoh7 - servo1, 3

# Software setup
## radxa 
1. `ssh USER@IP_ADRESS`
2. Connect to master PC  
`rossetmaster MASTER_IP_ADDRESS`
4. Run arm controller  
  `$ roslaunch umoru_arm umoru_arm.launch arm:="rarm"`

## master PC
1. Start roscore
2. Publish ros messages for each arm
- Velocity (actually represents duration) ranges from 1 to 255.
```
$ rostopic pub /umoru_rarm_controller/joint_state sensor_msgs/JointState "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
name: ['joint_yaw']
position: [0]
velocity: [120]
effort: [0]" -1
```
