#! /usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from inverse_problem_srv.srv import point_cmd, point_cmdResponse
#from main_to_point_node import ParseMsg, 
from std_srvs.srv import SetBool
import sys

rospy.init_node("angle_robot_move_node")




_x_ = 0
_y_ = 1
_z_ = 2
_phi_ = 3

_open_ = True
_close_ = False

def move2point_cb(msg):
    print(msg)

srv_ang_pose = rospy.ServiceProxy("/angle_robot/cmd_point", point_cmd)
srv_ang_grip = rospy.ServiceProxy("/angle_robot/gripper_cmd", SetBool)





def cors2str(cors):
    return str(cors[_x_] + 231.257) + " " + str(cors[_y_]) + " " + str(cors[_z_]) + " " + str(cors[_phi_])


def cors2str_upper(cors):
    return str(cors[_x_] + 231.257) + " " + str(cors[_y_] ) + " 70 " + str(cors[_phi_])


wsh_pos = [-431.257, 0, 40, 0]
start_pos = [-360.257, 0, 40, 0]


def move2UPpoint(point):
    new_pos = cors2str_upper(point)
    
    res = srv_ang_pose(new_pos)  
    
    if not res.result:
        rospy.logfatal("GO TO POS: " + str(new_pos) + " " + str(res))
        sys.exit()
    else:
        rospy.loginfo("GO TO POS: " + str(new_pos) + " " + str(res))

def move2DOWNpoint(point):
    new_pos = cors2str(point)
    res = srv_ang_pose(new_pos)  
    
    if not res.result:
        rospy.logfatal("GO TO POS: " + str(new_pos) + " " + str(res))
        sys.exit()
    else:
        rospy.loginfo("GO TO POS: " + str(new_pos) + " " + str(res))

move2UPpoint(start_pos)

def proccess():
    print("############CHOOSE A CUBE COLOR##########:")
    print(rospy.get_param("/defined_cube_colors"))
    color = input(str)
    if rospy.has_param("/find_cubes/" + color + "/"):
        rospy.loginfo("FIND PARAM /find_cubes/" + color + "/")
        counter = 0
        while(1):
            if rospy.has_param("/find_cubes/" + color + "/" + str(counter) + "/pos"):
                rospy.loginfo("FIND PARAM /find_cubes/" + color + "/" + str(counter) + "/pos")
                cors = rospy.get_param("/find_cubes/" + color + "/" + str(counter) + "/pos")
                
                print(str(cors))
                
                move2UPpoint(cors) 

                srv_ang_grip(_open_)
                rospy.loginfo("GRIPPER OPEN" )

                move2DOWNpoint(cors)

                srv_ang_grip(_close_)
                rospy.loginfo("GRIPPER CLOSE" )

                move2UPpoint(cors) 
                
                move2UPpoint(wsh_pos)

                move2DOWNpoint(wsh_pos)

                srv_ang_grip(_open_)
                rospy.loginfo("GRIPPER OPEN" )

                move2UPpoint(start_pos)

                rospy.delete_param("/find_cubes/" + color + "/" + str(counter) + "/pos")
                counter = counter + 1
                wsh_pos[_z_] = wsh_pos[_z_] + 30
                
                
            else:
                break
    
                

if __name__ == "__main__":
    while not rospy.is_shutdown():
        proccess()
        
    
