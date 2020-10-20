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

_ON_ = True
_OFF_ = False

def move2point_cb(msg):
    print(msg)

srv_pal_pose = rospy.ServiceProxy("/palletizer_robot/cmd_point", point_cmd)
srv_pal_grip = rospy.ServiceProxy("/palletizer_robot/vacuum", SetBool)





def cors2str(cors):
    return str(cors[_x_] + 200) + " " + str(-cors[_y_]+5) + " " + str(cors[_z_]+2)


def cors2str_upper(cors):
    return str(cors[_x_] + 200) + " " + str(-cors[_y_]+5) + " 70 " 


wsh_pos = [-431.257, 0, 40, 0]
start_pos = [-360.257, 0, 40, 0]


def move2UPpoint(point):
    new_pos = cors2str_upper(point)
    
    res = srv_pal_pose(new_pos)  
    
    if not res.result:
        rospy.logerr("GO TO POS: " + str(new_pos) + " " + str(res))
    else:
        rospy.loginfo("GO TO POS: " + str(new_pos) + " " + str(res))

def move2DOWNpoint(point):
    new_pos = cors2str(point)
    res = srv_pal_pose(new_pos)  
    
    if not res.result:
        rospy.logerr("GO TO POS: " + str(new_pos) + " " + str(res))
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

                move2DOWNpoint(cors)

                srv_pal_grip(_ON_)
                rospy.loginfo("VACUUM ON" )

                move2UPpoint(cors) 
                
                move2UPpoint(wsh_pos)

                move2DOWNpoint(wsh_pos)

                srv_pal_grip(_OFF_)
                rospy.loginfo("VACUUM OFF" )

                move2UPpoint(start_pos)

                rospy.delete_param("/find_cubes/" + color + "/" + str(counter) + "/pos")
                counter = counter + 1
                wsh_pos[_z_] = wsh_pos[_z_] + 30
                
                
            else:
                break
    
                

if __name__ == "__main__":
    while not rospy.is_shutdown():
        proccess()
        
    
