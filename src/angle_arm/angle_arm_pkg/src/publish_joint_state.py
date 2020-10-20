#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import math
from inverse_problem_srv.srv import publish_cmd,publish_cmdResponse
import numpy as np

jointPub = rospy.Publisher('/angle_robot/joint_states_kinematic',JointState,queue_size = 100)

currentState = JointState()
last_gripper = 0
currentState.position = [0,0,0,0,0,0]
maxVelocity = 1.5
gripper_vel = 2
gripper_pos = 20
countOfJoint = 6
iter_time = 0.01
gripperEf = 0

def realize_of_principle(goalPoseMsg,jointName,timeOfWay):
    global currentState
    q = []
    time = []
    time.append(0)
    i = 0
    k=0
    start = np.array(currentState.position[0:countOfJoint-1])
    end = np.array(goalPoseMsg)
    const = (end-start)
    q.append(start)
    while(i<=timeOfWay):
        i+=iter_time
        if(i<timeOfWay/3):
            vel = 9*i*const/(2*timeOfWay**2)
        elif(i>2*timeOfWay/3):
            vel = (-9/(2*timeOfWay**2)*i+9/(2*timeOfWay))*const
        else:
            vel = 3/(2*timeOfWay)*const
        q.append(q[k] + vel*iter_time)
        k+=1
        time.append(i)
    q_command = []

    for i in range(countOfJoint-1):
        qi = [r[i] for r in q]
        q_command.append(qi)
    q = q_command
    jointMsg = JointState()
    jointMsg.name = jointName
    for i in range(len(q_command[0])):
        jointMsg.position = [q[0][i],q[1][i],q[2][i],q[3][i],q[4][i],last_gripper]
        jointMsg.header.stamp = rospy.Time.now()
        jointPub.publish(jointMsg)
        rospy.sleep(iter_time)
    currentState.position = goalPoseMsg

def way_compute(posList):
    global currentState
    wayList = []
    for i in range(len(posList)):
        wayList.append(posList[i]-currentState.position[i])
    return wayList

def max_way(wayList):
    maxWay = wayList[len(wayList)-1]
    for i in range(len(wayList)-1):
        if (math.fabs(wayList[i])>math.fabs(maxWay)):
            maxWay = wayList[i]
        else:
            continue
    return maxWay

def parse_msg(msg):
    jointState = JointState()
    msgList = msg.jointState.split()
    jointState.name = msgList[0:countOfJoint]
    jointState.position = msgList[countOfJoint:]
    jointState.position = [float(el) for el in jointState.position] 
    return jointState

def move_of_trapeze_principle(msg):
    global last_gripper
    msg = parse_msg(msg)
    gripper = msg.position[countOfJoint-1]
    way_list = way_compute(msg.position[0:countOfJoint-1])
    max_way_new = max_way(way_list)
    if(not max_way_new == 0):
        time = 3*math.fabs(max_way_new)/(2*maxVelocity)
        realize_of_principle(msg.position[0:countOfJoint-1],msg.name,time)
    else:
        rate = rospy.Rate(10)
        if(gripper == last_gripper):
            for i in range(15):
                msg.header.stamp = rospy.Time.now()
                jointPub.publish(msg)
                rate.sleep()
        else:
            jointMsg = JointState()
            if(gripper == 1):
                current_pose = last_gripper
                while(current_pose <= gripper_pos):
                    current_pose+=gripper_vel*iter_time*10
                    jointMsg.position = [msg.position[0],msg.position[1],msg.position[2],msg.position[3],msg.position[4],current_pose]
                    jointMsg.header.stamp = rospy.Time.now()
                    jointPub.publish(jointMsg)
                    rospy.sleep(iter_time)
                    if(gripperEf > 4):
                        break
            elif(gripper==0):
                current_pose = last_gripper
                while(current_pose >= 0):
                    current_pose-=gripper_vel*iter_time*10
                    jointMsg.position = [msg.position[0],msg.position[1],msg.position[2],msg.position[3],msg.position[4],current_pose]
                    jointMsg.header.stamp = rospy.Time.now()
                    jointPub.publish(jointMsg)
                    rospy.sleep(iter_time)
                    if(gripperEf > 4):
                        break
            last_gripper = current_pose
    print("true")
    return publish_cmdResponse(True)
     
def updateEffort(msg):
    global gripperEf
    gripperEf = math.fabs(msg.effort[0])

if __name__=='__main__':
    rospy.init_node('angle_convert_and_publish_state')
    rospy.loginfo('Start_convert_and_publish_state_node')
    rospy.Service('/angle_robot/cmd_joint_state_in_manip_coord', publish_cmd, move_of_trapeze_principle)
    rospy.Subscriber('/angle_robot/joint_states',JointState,updateEffort)
    rospy.spin()
