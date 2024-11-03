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
                namespace + "/robot_description_viz"
            )
            self.r_ri = KXRROSRobotInterface(  # NOQA
                self.r_robot_model, namespace=namespace, controller_timeout=60.0
            )
            self.r_robot_model.angle_vector(self.r_ri.angle_vector())
            self.r_ri.servo_on()
        if arm == "both" or arm == "larm":
            namespace = "larm"
            self.l_robot_model = RobotModel()
            self.l_robot_model.load_urdf_from_robot_description(
                namespace + "/robot_description_viz"
            )
            self.l_ri = KXRROSRobotInterface(  # NOQA
                self.l_robot_model, namespace=namespace, controller_timeout=60.0
            )
            self.l_robot_model.angle_vector(self.l_ri.angle_vector())
            self.l_ri.servo_on()

    def reset_pose(self, velocity=300):
        if self.mode == "both" or self.mode == "rarm":
            self.r_robot_model.joint_pitch.joint_angle(np.deg2rad(0))
            self.r_robot_model.joint_yaw.joint_angle(np.deg2rad(-135))
            self.r_ri.angle_vector(self.r_robot_model.angle_vector(), time=1.5)
        if self.mode == "both" or self.mode == "larm":
            self.l_robot_model.joint_pitch.joint_angle(np.deg2rad(0))
            self.l_robot_model.joint_yaw.joint_angle(np.deg2rad(135))
            self.l_ri.angle_vector(self.l_robot_model.angle_vector(), time=1.5)
        while self.l_ri.is_interpolating() or self.r_ri.is_interpolating():
            rospy.sleep(0.1)

    def init_pose(self, velocity=300):
        if self.mode == "both" or self.mode == "rarm":
            self.r_robot_model.joint_pitch.joint_angle(np.deg2rad(90))
            self.r_robot_model.joint_yaw.joint_angle(np.deg2rad(-135))
            self.r_ri.angle_vector(self.r_robot_model.angle_vector(), time=1.5)
        if self.mode == "both" or self.mode == "larm":
            self.l_robot_model.joint_pitch.joint_angle(np.deg2rad(-90))
            self.l_robot_model.joint_yaw.joint_angle(np.deg2rad(135))
            self.l_ri.angle_vector(self.l_robot_model.angle_vector(), time=1.5)
        while self.l_ri.is_interpolating() or self.r_ri.is_interpolating():
            rospy.sleep(0.1)

    def hug(self, velocity=300):
        if self.mode == "both" or self.mode == "rarm":
            self.r_robot_model.joint_yaw.joint_angle(np.deg2rad(-50))
            self.r_ri.angle_vector(self.r_robot_model.angle_vector(), time=1.5)
        if self.mode == "both" or self.mode == "larm":
            self.l_robot_model.joint_yaw.joint_angle(np.deg2rad(50))
            self.l_ri.angle_vector(self.l_robot_model.angle_vector(), time=1.5)
        while self.l_ri.is_interpolating() or self.r_ri.is_interpolating():
            rospy.sleep(0.1)

    def extend(self, velocity=300):
        if self.mode == "both" or self.mode == "rarm":
            self.r_robot_model.joint_yaw.joint_angle(np.deg2rad(-135))
            self.r_ri.angle_vector(self.r_robot_model.angle_vector(), time=1.5)
        if self.mode == "both" or self.mode == "larm":
            self.l_robot_model.joint_yaw.joint_angle(np.deg2rad(135))
            self.l_ri.angle_vector(self.l_robot_model.angle_vector(), time=1.5)
        while self.l_ri.is_interpolating() or self.r_ri.is_interpolating():
            rospy.sleep(0.1)

    def raise_arm(self, arm, velocity=300):
        if self.mode == "both" or self.mode == "rarm":
            self.r_robot_model.joint_pitch.joint_angle(np.deg2rad(-90))
            self.r_robot_model.joint_yaw.joint_angle(np.deg2rad(-135))
            self.r_ri.angle_vector(self.r_robot_model.angle_vector(), time=1.5)
        if self.mode == "both" or self.mode == "larm":
            self.l_robot_model.joint_pitch.joint_angle(np.deg2rad(90))
            self.l_robot_model.joint_yaw.joint_angle(np.deg2rad(135))
            self.l_ri.angle_vector(self.l_robot_model.angle_vector(), time=1.5)
        while self.l_ri.is_interpolating() or self.r_ri.is_interpolating():
            rospy.sleep(0.1)

