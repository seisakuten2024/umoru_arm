#!/usr/bin/env python3

import numpy as np
import rospy
from rcb4.armh7interface import ARMH7Interface
from sensor_msgs.msg import JointState

class UmoruArmController():
    def __init__(self):
        self.joint_name_to_id = {'joint_pitch':1, 'joint_yaw':3}
        self.interface = ARMH7Interface()
        device = rospy.get_param('~device', '')
        if device == '':
            self.interface.auto_open()
        else:
            self.interface.open(device)

        self.sub = rospy.Subscriber('~joint_state',
            JointState, queue_size=1,
            callback=self.callback)

        rospy.loginfo("id: {}".format(self.interface.search_servo_ids()))

    def callback(self, msg):
        servo_ids = []
        angle_vector = []
        velocity_vector = []
        for name, position, velocity in zip(msg.name, msg.position, msg.velocity):
            if name not in self.joint_name_to_id:
                continue
            idx = self.joint_name_to_id[name]
            servo_ids.append(idx)
            angle_vector.append(position)
            velocity_vector.append(velocity)
            
        angle_vector = np.array(angle_vector)
        velocity_vector = np.array(velocity_vector)
        servo_ids = np.array(servo_ids, dtype=np.int32)
        self.interface.angle_vector(angle_vector, servo_ids, velocity_vector)

if __name__ == '__main__':
    rospy.init_node('umoru_arm_controller')
    umoru_arm_controller = UmoruArmController()
    rospy.spin()
