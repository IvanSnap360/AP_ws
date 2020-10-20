#! /usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from inverse_problem_srv.srv import point_cmd
from std_srvs.srv import SetBool

rospy.init_node("palletizer_robot_move_node")

_x_ = 0
_y_ = 1
_z_ = 2
_phi_ = 3
_ON_ = True
_OFF_ = False

srv_pal_pose = rospy.ServiceProxy("/palletizer_robot/cmd_point", point_cmd)
srv_pal_pris = rospy.ServiceProxy("/palletizer_robot/vacuum", SetBool)


def cors2str(cors):
    return str(cors[_x_] - 231.257) + " " + str(-cors[_y_]) + " " + str(cors[_z_]-20) + " " + str(0)


def cors2str_upper(cors):
    return str(cors[_x_] - 231.257) + " " + str(-cors[_y_]) + " 90 " + str(cors[_phi_])
    
wsh_pos = [-400.257, 0, 40, 1.57]
star_pos = wsh_pos
srv_pal_pose(cors2str_upper(star_pos))




def proccess():
    print("############CHOOSE A CUBE COLOR##########:")
    print(rospy.get_param("/defined_cube_colors"))
    color = input(str)
    if rospy.has_param("/find_cubes/" + color + "/"):
        print("FIND PARAM","/find_cubes/" + color + "/")
        counter = 0
        while(1):
            if rospy.has_param("/find_cubes/" + color + "/" + str(counter) + "/pos"):
                print("FIND PARAM","/find_cubes/" + color + "/" + str(counter) + "/pos")
                cors = rospy.get_param("/find_cubes/" + color + "/" + str(counter) + "/pos")
                rospy.delete_param("/find_cubes/" + color + "/" + str(counter) + "/pos")
                print(str(cors))
                
                srv_pal_pris(_OFF_)
                print("PRISOS OFF")

                new_pos = cors2str_upper(cors)
                print("GO TO POS: " + str(new_pos))
                srv_pal_pose(new_pos)
                
                new_pos = cors2str(cors)
                print("GO TO POS: " + str(new_pos))
                srv_pal_pose(new_pos)

                srv_pal_pris(_ON_)
                print("PRISOS ON")

                new_pos = cors2str_upper(cors)
                print("GO TO POS: " + str(new_pos))
                srv_pal_pose(new_pos)

                new_pos = cors2str_upper(wsh_pos)
                print("GO TO POS: " + str(new_pos))
                srv_pal_pose(new_pos)
                
                new_pos = cors2str(wsh_pos)
                print("GO TO POS: " + str(new_pos))
                srv_pal_pose(new_pos)    

                srv_pal_pris(_OFF_)
                print("PRISOS OFF")

                new_pos = cors2str_upper(wsh_pos)
                print("GO TO POS: " + str(new_pos))
                srv_pal_pose(new_pos)

                counter = counter + 1
                wsh_pos[_y_] = wsh_pos[_y_] + 100
                
            else:
                break

                

if __name__ == "__main__":
    while not rospy.is_shutdown():
        proccess()
        
    
