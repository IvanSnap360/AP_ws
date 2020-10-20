#!/usr/bin/env python3
import rospy
import os
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty

rospy.wait_for_service('gazebo/spawn_urdf_model')
spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)
del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
cubs_status = False

def init_polygon():
    global cubs_status
    pose_red = Pose()
    pose_green = Pose()
    pose_yellow = Pose()
    pose_blue = Pose()

    pose_red.position.z = pose_green.position.z = 0.15
    pose_red.position.x = 0.1
    pose_green.position.x = 0.2

    pose_yellow.position.x = 0.2
    pose_yellow.position.y = -0.1
    pose_yellow.position.z = 0.15
    
    pose_blue.position.x = -0.2
    pose_blue.position.y = -0.1
    pose_blue.position.z = 0.15

    cub_model_f = open(cub_model_path,'r')
    cub_model = cub_model_f.read()

    if(not cubs_status):
        red_cub_model = cub_model.replace('Color','Red')
        spawn_model_prox('cub_red', red_cub_model, "object", pose_red, "world")

        green_cub_model = cub_model.replace('Color','Green')
        spawn_model_prox('cub_green', green_cub_model, "object", pose_green, "world")

        yellow_cub_model = cub_model.replace('Color','Yellow')
        spawn_model_prox('cub_yellow', yellow_cub_model, "object", pose_yellow, "world")
        
        blue_cub_model = cub_model.replace('Color','Blue')
        spawn_model_prox('cub_blue', blue_cub_model, "object", pose_blue, "world")

        cubs_status = True
    

def restart_polygon(msg):
    global cubs_status
    if(cubs_status):
        delete_cubs('')
        cubs_status = False
    init_polygon()
    return msg

def delete_cubs(msg):
    global cubs_status
    if(cubs_status):
        del_model_prox('cub_red')
        del_model_prox('cub_green')
        cubs_status = False
    return msg

if __name__=='__main__':
    global cub_model_path
    rospy.init_node('cube_spawn')

    model_dir = os.path.dirname(os.path.realpath(__file__))
    model_dir = model_dir[0:-3]
    model_dir = model_dir + 'urdf/'
    cub_model_path = model_dir+'cubs.urdf'
    init_polygon()
    rospy.Service('/polygon/restart_cubs',Empty,restart_polygon)
    rospy.Service('/polygon/delete_cubs',Empty,delete_cubs)
    rospy.spin()
