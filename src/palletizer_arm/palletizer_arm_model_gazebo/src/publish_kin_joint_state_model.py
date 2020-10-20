#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

joint0 = rospy.Publisher('/palletizer_robot/pal_joint0_controller/command',Float64,queue_size=10)
joint1 = rospy.Publisher('/palletizer_robot/pal_joint1_controller/command',Float64,queue_size=10)
joint2 = rospy.Publisher('/palletizer_robot/pal_joint2_controller/command',Float64,queue_size=10)
joint3 = rospy.Publisher('/palletizer_robot/pal_joint3_controller/command',Float64,queue_size=10)
joint4 = rospy.Publisher('/palletizer_robot/pal_joint4_controller/command',Float64,queue_size=10)
joint5 = rospy.Publisher('/palletizer_robot/pal_joint5_controller/command',Float64,queue_size=10)
joint6 = rospy.Publisher('/palletizer_robot/pal_joint6_controller/command',Float64,queue_size=10)
joint7 = rospy.Publisher('/palletizer_robot/pal_joint7_controller/command',Float64,queue_size=10)
joint8 = rospy.Publisher('/palletizer_robot/pal_joint8_controller/command',Float64,queue_size=10)
joint9 = rospy.Publisher('/palletizer_robot/pal_joint9_controller/command',Float64,queue_size=10)
prisos = rospy.Publisher('/palletizer_robot/pal_joint_prisos_controller/command',Float64,queue_size=10)

def main_remap(jointState):
    joint0.publish(jointState.position[0])
    joint1.publish(jointState.position[1])
    joint2.publish(jointState.position[2])
    joint3.publish(jointState.position[3])
    prisos.publish(jointState.position[4])
    joint4.publish(jointState.position[2])
    joint9.publish(-jointState.position[1])
    joint5.publish(jointState.position[1])
    joint6.publish(-jointState.position[2])
    joint7.publish(-jointState.position[2]-jointState.position[1])
    joint8.publish(jointState.position[2]+jointState.position[1])

if __name__=="__main__":
    rospy.init_node('palletizer_publish_kin_joint_state_to_model')
    rospy.Subscriber("/palletizer_robot/joint_states_kinematic", JointState, main_remap)
    rospy.spin()
