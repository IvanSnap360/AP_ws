#!/usr/bin/env python3
import rospy
import os
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
from std_srvs.srv import SetBool

pal_red_light_pose = Pose()
pal_green_light_pose = Pose()
pal_yellow_light_pose = Pose()
pal_blue_light_pose = Pose()

ang_red_light_pose = Pose()
ang_green_light_pose = Pose()
ang_yellow_light_pose = Pose()
ang_blue_light_pose = Pose()

pal_red_light_on = False
pal_green_light_on = False
pal_yellow_light_on = False
pal_blue_light_on = False

ang_red_light_on = False
ang_green_light_on = False
ang_yellow_light_on = False
ang_blue_light_on = False

rospy.wait_for_service('gazebo/spawn_sdf_model')
spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)

def init_pose_light():
    global ang_red_light_pose,ang_green_light_pose,ang_yellow_light_pose,ang_blue_light_pose,pal_red_light_pose,pal_green_light_pose,pal_yellow_light_pose,pal_blue_light_pose

    ang_red_light_pose.position.x = ang_green_light_pose.position.x = ang_yellow_light_pose.position.x = ang_blue_light_pose.position.x = 0.292
    pal_red_light_pose.position.x = pal_green_light_pose.position.x = pal_yellow_light_pose.position.x = pal_blue_light_pose.position.x = 0.292

    ang_red_light_pose.position.y = ang_green_light_pose.position.y = ang_yellow_light_pose.position.y = ang_blue_light_pose.position.y = -0.415
    pal_red_light_pose.position.y = pal_green_light_pose.position.y = pal_yellow_light_pose.position.y = pal_blue_light_pose.position.y = 0.415

    ang_red_light_pose.position.z = pal_red_light_pose.position.z = 0.48
    ang_yellow_light_pose.position.z = pal_yellow_light_pose.position.z = 0.44
    ang_green_light_pose.position.z = pal_green_light_pose.position.z = 0.40
    ang_blue_light_pose.position.z = pal_blue_light_pose.position.z = 0.36

def set_angle_red_light(msg):
    global ang_red_light_on
    if(msg.data):
        if(not ang_red_light_on):
            spawn_model_prox('red_light_ang', red_light_model, "object", ang_red_light_pose, "world")
            ang_red_light_on = True
    else:
        if(ang_red_light_on):
            del_model_prox('red_light_ang')
            ang_red_light_on = False
    return True, 'Success'

def set_angle_yellow_light(msg):
    global ang_yellow_light_on
    if(msg.data):
        if(not ang_yellow_light_on):
            spawn_model_prox('yellow_light_ang', yellow_light_model, "object", ang_yellow_light_pose, "world")
            ang_yellow_light_on = True
    else:
        if(ang_yellow_light_on):
            del_model_prox('yellow_light_ang')
            ang_yellow_light_on = False
    return True, 'Success'

def set_angle_green_light(msg):
    global ang_green_light_on
    if(msg.data):
        if(not ang_green_light_on):
            spawn_model_prox('green_light_ang', green_light_model, "object", ang_green_light_pose, "world")
            ang_green_light_on = True
    else:
        if(ang_green_light_on):
            del_model_prox('green_light_ang')
            ang_green_light_on = False
    return True, 'Success'

def set_angle_blue_light(msg):
    global ang_blue_light_on
    if(msg.data):
        if(not ang_blue_light_on):
            spawn_model_prox('blue_light_ang', blue_light_model, "object", ang_blue_light_pose, "world")
            ang_blue_light_on = True
    else:
        if(ang_blue_light_on):
            del_model_prox('blue_light_ang')
            ang_blue_light_on = False
    return True, 'Success'

def set_pal_red_light(msg):
    global pal_red_light_on
    if(msg.data):
        if(not pal_red_light_on):
            spawn_model_prox('red_light_pal', red_light_model, "object", pal_red_light_pose, "world")
            pal_red_light_on = True
    else:
        if(pal_red_light_on):
            del_model_prox('red_light_pal')
            pal_red_light_on = False
    return True, 'Success'

def set_pal_yellow_light(msg):
    global pal_yellow_light_on
    if(msg.data):
        if(not pal_yellow_light_on):
            spawn_model_prox('yellow_light_pal', yellow_light_model, "object", pal_yellow_light_pose, "world")
            pal_yellow_light_on = True
    else:
        if(pal_yellow_light_on):
            del_model_prox('yellow_light_pal')
            pal_yellow_light_on = False
    return True, 'Success'

def set_pal_green_light(msg):
    global pal_green_light_on
    if(msg.data):
        if(not pal_green_light_on):
            spawn_model_prox('green_light_pal', green_light_model, "object", pal_green_light_pose, "world")
            pal_green_light_on = True
    else:
        if(pal_green_light_on):
            del_model_prox('green_light_pal')
            pal_green_light_on = False
    return True, 'Success'

def set_pal_blue_light(msg):
    global pal_blue_light_on
    if(msg.data):
        if(not pal_blue_light_on):
            spawn_model_prox('blue_light_pal', blue_light_model, "object", pal_blue_light_pose, "world")
            pal_blue_light_on = True
    else:
        if(pal_blue_light_on):
            del_model_prox('blue_light_pal')
            pal_blue_light_on = False

    return True, 'Success'

if __name__=='__main__':
    global red_light_model,yellow_light_model,green_light_model,blue_light_model

    rospy.init_node('light')
    init_pose_light()

    model_dir = os.path.dirname(os.path.realpath(__file__))
    model_dir = model_dir[0:-3]
    model_dir = model_dir + 'urdf/'

    red_light_path = model_dir+'red_light.urdf'
    green_light_path = model_dir+'green_light.urdf'
    yellow_light_path = model_dir+'yellow_light.urdf'
    blue_light_path = model_dir+'blue_light.urdf'

    red_light_model = open(red_light_path,'r')
    red_light_model = red_light_model.read()

    green_light_model = open(green_light_path,'r')
    green_light_model = green_light_model.read()

    yellow_light_model = open(yellow_light_path,'r')
    yellow_light_model = yellow_light_model.read()
    
    blue_light_model = open(blue_light_path,'r')
    blue_light_model = blue_light_model.read()

    rospy.Service('/angle_robot/red_light',SetBool,set_angle_red_light)
    rospy.Service('/angle_robot/yellow_light',SetBool,set_angle_yellow_light)
    rospy.Service('/angle_robot/green_light',SetBool,set_angle_green_light)
    rospy.Service('/angle_robot/blue_light',SetBool,set_angle_blue_light)
    rospy.Service('/palletizer_robot/red_light',SetBool,set_pal_red_light)
    rospy.Service('/palletizer_robot/yellow_light',SetBool,set_pal_yellow_light)
    rospy.Service('/palletizer_robot/green_light',SetBool,set_pal_green_light)
    rospy.Service('/palletizer_robot/blue_light',SetBool,set_pal_blue_light)
    rospy.spin()
