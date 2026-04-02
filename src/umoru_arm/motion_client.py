import rospy
from sensor_msgs.msg import JointState
import numpy as np  # NOQA
from skrobot.model import RobotModel
from kxr_controller.kxr_interface import KXRROSRobotInterface

class MotionClient(object):
    def __init__(self, arm):
        self.mode = arm
        if arm == "both" or arm == "rarm":
            namespace = "rarm"
            self.r_robot_model = RobotModel()
            self.r_robot_model.load_urdf_from_robot_description(
                namespace + "/robot_description"
            )
            self.r_ri = KXRROSRobotInterface(  # NOQA
                self.r_robot_model, namespace=namespace, controller_timeout=60.0
            )
            self.r_robot_model.angle_vector(self.r_ri.angle_vector())
            self.r_ri.send_stretch(127)
            self.r_ri.servo_on()
        if arm == "both" or arm == "larm":
            namespace = "larm"
            self.l_robot_model = RobotModel()
            self.l_robot_model.load_urdf_from_robot_description(
                namespace + "/robot_description"
            )
            self.l_ri = KXRROSRobotInterface(  # NOQA
                self.l_robot_model, namespace=namespace, controller_timeout=60.0
            )
            self.l_robot_model.angle_vector(self.l_ri.angle_vector())
            self.l_ri.send_stretch(127)
            self.l_ri.servo_on()

    def wait_interpolation(self):
        if self.mode == "both":
            while self.l_ri.is_interpolating() and self.r_ri.is_interpolating():
                rospy.sleep(0.1)
        elif self.mode == "larm":
            while self.l_ri.is_interpolating():
                rospy.sleep(0.1)
        elif self.mode == "rarm":
            while self.r_ri.is_interpolating():
                rospy.sleep(0.1)

    def reset_pose(self, velocity=300):
        if self.mode == "both" or self.mode == "rarm":
            self.r_robot_model.joint_pitch.joint_angle(np.deg2rad(0))
            self.r_robot_model.joint_yaw.joint_angle(np.deg2rad(-135))
            self.r_ri.angle_vector(self.r_robot_model.angle_vector(), time=1.5)
        if self.mode == "both" or self.mode == "larm":
            self.l_robot_model.joint_pitch.joint_angle(np.deg2rad(0))
            self.l_robot_model.joint_yaw.joint_angle(np.deg2rad(135))
            self.l_ri.angle_vector(self.l_robot_model.angle_vector(), time=1.5)
        self.wait_interpolation()

    def init_pose(self, velocity=300):
        if self.mode == "both" or self.mode == "rarm":
            self.r_robot_model.joint_pitch.joint_angle(np.deg2rad(90))
            self.r_robot_model.joint_yaw.joint_angle(np.deg2rad(-135))
            self.r_ri.angle_vector(self.r_robot_model.angle_vector(), time=1.5)
        if self.mode == "both" or self.mode == "larm":
            self.l_robot_model.joint_pitch.joint_angle(np.deg2rad(-90))
            self.l_robot_model.joint_yaw.joint_angle(np.deg2rad(135))
            self.l_ri.angle_vector(self.l_robot_model.angle_vector(), time=1.5)
        self.wait_interpolation()

    def hug(self, velocity=500):
        if self.mode == "both" or self.mode == "rarm":
            self.r_robot_model.joint_yaw.joint_angle(np.deg2rad(-60))
            self.r_ri.angle_vector(self.r_robot_model.angle_vector(), time=1.5)
        if self.mode == "both" or self.mode == "larm":
            self.l_robot_model.joint_yaw.joint_angle(np.deg2rad(20))
            self.l_ri.angle_vector(self.l_robot_model.angle_vector(), time=1.5)
        self.wait_interpolation()

    def extend(self, velocity=500):
        if self.mode == "both" or self.mode == "rarm":
            self.r_robot_model.joint_yaw.joint_angle(np.deg2rad(-135))
            self.r_ri.angle_vector(self.r_robot_model.angle_vector(), time=1.5)
        if self.mode == "both" or self.mode == "larm":
            self.l_robot_model.joint_yaw.joint_angle(np.deg2rad(135))
            self.l_ri.angle_vector(self.l_robot_model.angle_vector(), time=1.5)
        self.wait_interpolation()

    def raise_arm(self, arm, velocity=300):
        if self.mode == "both" or self.mode == "rarm":
            self.r_robot_model.joint_pitch.joint_angle(np.deg2rad(-90))
            self.r_robot_model.joint_yaw.joint_angle(np.deg2rad(-135))
            self.r_ri.angle_vector(self.r_robot_model.angle_vector(), time=1.5)
        if self.mode == "both" or self.mode == "larm":
            self.l_robot_model.joint_pitch.joint_angle(np.deg2rad(90))
            self.l_robot_model.joint_yaw.joint_angle(np.deg2rad(135))
            self.l_ri.angle_vector(self.l_robot_model.angle_vector(), time=1.5)
        self.wait_interpolation()
