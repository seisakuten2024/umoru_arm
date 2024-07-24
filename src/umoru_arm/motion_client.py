import rospy
from sensor_msgs.msg import JointState

class MotionClient(object):
    def __init__(self, arm):
        self.mode = arm
        if arm == "both" or arm == "rarm":
            self.rarm_pub = rospy.Publisher('/umoru_rarm_controller/joint_state', JointState, queue_size=1)
        if arm == "both" or arm == "larm":
            self.larm_pub = rospy.Publisher('/umoru_larm_controller/joint_state', JointState, queue_size=1)


    def angle_vector(self, arm, name, position, velocity):
        msg = JointState()
        msg.name = name
        msg.position = position
        msg.velocity = velocity
        if arm == "rarm":
            self.rarm_pub.publish(msg)


    def reset_pose(self, velocity=300):
        msg = JointState()
        msg.name = ['joint_pitch', 'joint_yaw']
        msg.velocity = [velocity, velocity]
        if self.mode == "both" or self.mode == "rarm":
            msg.position = [0, -135]
            self.rarm_pub.publish(msg)


    def init_pose(self, velocity=300):
        msg = JointState()
        msg.name = ['joint_pitch', 'joint_yaw']
        msg.velocity = [velocity, velocity]
        if self.mode == "both" or self.mode == "rarm":
            msg.position = [90, -135]
            self.rarm_pub.publish(msg)


    def hug(self, velocity=300):
        msg = JointState()
        msg.name = ['joint_yaw']
        msg.velocity = [velocity]
        if self.mode == "both" or self.mode == "rarm":
            msg.position = [-50]
            self.rarm_pub.publish(msg)


    def extend(self, velocity=300):
        msg = JointState()
        msg.name = ['joint_yaw']
        msg.velocity = [velocity]
        if self.mode == "both" or self.mode == "rarm":
            msg.position = [-135]
            self.rarm_pub.publish(msg)


    def raise_arm(self, arm, velocity=300):
        msg = JointState()
        msg.name = ['joint_pitch']
        msg.velocity = [velocity]
        if arm == "both" or arm == "rarm":
            msg.position = [-90]
            self.rarm_pub.publish(msg)


