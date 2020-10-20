#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

joint1 = rospy.Publisher('/angle_robot/angle_joint1_controller/command',Float64,queue_size=10)
joint2 = rospy.Publisher('/angle_robot/angle_joint2_controller/command',Float64,queue_size=10)
joint3 = rospy.Publisher('/angle_robot/angle_joint3_controller/command',Float64,queue_size=10)
joint4 = rospy.Publisher('/angle_robot/angle_joint4_controller/command',Float64,queue_size=10)
joint5 = rospy.Publisher('/angle_robot/angle_joint5_controller/command',Float64,queue_size=10)
finger_l = rospy.Publisher('/angle_robot/angle_gripper_controller/command',Float64,queue_size=10)
finger_r = rospy.Publisher('/angle_robot/angle_gripper_sub_controller/command',Float64,queue_size=10)

def sendJointStateToModel(msg):
    joint1.publish(msg.position[0])
    joint2.publish(msg.position[1])
    joint3.publish(msg.position[2])
    joint4.publish(msg.position[3])
    joint5.publish(msg.position[4])
    finger_l.publish(msg.position[5]/1000)
    finger_r.publish(msg.position[5]/1000)


if __name__=="__main__":
    rospy.init_node('angle_publish_kin_joint_state_to_model')
    rospy.Subscriber('/angle_robot/joint_states_kinematic',JointState,sendJointStateToModel)
    rospy.spin()
