import rospy
from umoru_arm import MotionClient

rospy.init_node("test")
client = MotionClient("rarm")

client.reset_pose()

client.init_pose()
