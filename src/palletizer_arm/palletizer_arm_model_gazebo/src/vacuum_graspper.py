#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import LinkStates,LinkState
from gazebo_msgs.srv import ApplyBodyWrench
from math import sin,cos,sqrt
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist,Point, Wrench
import tf
import numpy as np

prisos_name = 'prisos_point'
pub = rospy.Publisher('/gazebo/set_link_state',LinkState,queue_size = 100)
force_srv = rospy.ServiceProxy('/gazebo/apply_body_wrench',ApplyBodyWrench)
a_box = 0.025
vacuum_on = False
pos_orient_prisos = 0
pose_cubs_list = []
id_cubs = []
link_name = []
twist = Twist()
last_attach_link = ''
force_on = False


def find_cubs_id(msg):
    id_cubs_list = []
    name_list = msg.name
    for i,el in enumerate(name_list):
        if(el[0:3] == "cub"):
            id_cubs_list.append(i)
    return id_cubs_list

def find_prisos_id(msg):
    for i,el in enumerate(msg.name):
        if(not el.find(prisos_name) == -1):
            return i
    return -1

def find_dists(cub_poses, prisos_pose):
    dists = []
    centre_box = []
    for cub_pos in cub_poses:
        (r_pris, p_pris, y_pris) = tf.transformations.euler_from_quaternion([prisos_pose.orientation.x, prisos_pose.orientation.y, prisos_pose.orientation.z, prisos_pose.orientation.w])
        (r_cub, p_cub, y_cub) = tf.transformations.euler_from_quaternion([cub_pos.orientation.x, cub_pos.orientation.y, cub_pos.orientation.z, cub_pos.orientation.w])

        prisos_trans = tf.transformations.compose_matrix(angles=[r_pris,p_pris,y_pris],translate=[prisos_pose.position.x,prisos_pose.position.y,prisos_pose.position.z])
        cub_trans = tf.transformations.compose_matrix(angles=[r_cub,p_cub,y_cub],translate=[cub_pos.position.x,cub_pos.position.y,cub_pos.position.z])
        prisos_trans_inv = tf.transformations.inverse_matrix(prisos_trans)
        cub2prisos = np.dot(prisos_trans_inv,cub_trans)
        _,_,angle,trans,_ = tf.transformations.decompose_matrix(cub2prisos)
        centre_in_cube = np.array([a_box/2,a_box/2,a_box/2,1])
        centre_cub_in_prisos = np.dot(cub2prisos,centre_in_cube)
        (x_q,y_q,z_q,w) = tf.transformations.quaternion_from_euler(angle[0],angle[1],angle[2])
        x = trans[0]
        y = trans[1]
        z = trans[2]
        centre_box.append(centre_cub_in_prisos[0:3])
        dists.append([x,y,z,x_q,y_q,z_q,w])
    return dists,centre_box

def find_aviable_cub(dists):
    for i,el in enumerate(dists):
        if(el[2]<0.016 and sqrt(el[0]**2 + el[1]**2)<0.016):
            return i
    return -1

def UpdateLinkStates(msg):
    global pos_orient_prisos,pose_cubs_list,id_cubs,link_name
    pose_cubs_list = []
    link_name = msg.name
    id_cubs = find_cubs_id(msg)
    if(len(id_cubs)==0):
        #print('cubs not found')
        return
    for id in id_cubs:
        pose_cubs_list.append(msg.pose[id])
    id_prisos = find_prisos_id(msg)

    pos_orient_prisos = msg.pose[id_prisos]

def main():
    global last_attach_link,force_on
    new_link_state = LinkState()
    force = Wrench()
    pose = Point()
    new_link_state.reference_frame = 'palletizer_robot::prisos_point'
    rate = rospy.Rate(100)
    dists = []
    centre_box = []
    while not rospy.is_shutdown():
        if(not vacuum_on):
            dists,centre_box = find_dists(pose_cubs_list,pos_orient_prisos)
            if(force_on):
                force.force.z = 0
                force_srv(last_attach_link,'world',pose,force,rospy.Time.from_sec(0),rospy.Duration.from_sec(-1.0))
                force_on = False
        else:
            id_grasp_cub = find_aviable_cub(centre_box)    
            if(not id_grasp_cub == -1):
                new_link_state.link_name = link_name[id_cubs[id_grasp_cub]]
                force.force.z = 0.01
                last_attach_link = link_name[id_cubs[id_grasp_cub]]
                dist = dists[id_grasp_cub]
                new_link_state.pose.orientation.x = dist[3]
                new_link_state.pose.orientation.y = dist[4]
                new_link_state.pose.orientation.z = dist[5]
                new_link_state.pose.orientation.w = dist[6]
                new_link_state.pose.position.x = dist[0]
                new_link_state.pose.position.y = dist[1]
                new_link_state.pose.position.z = dist[2]+0.003
                new_link_state.twist = twist
                pub.publish(new_link_state)
                if(not force_on):
                    force_srv(last_attach_link,'world',pose,force,rospy.Time.from_sec(0),rospy.Duration.from_sec(-1.0))
                    force_on = True
            else:
                dists,centre_box = find_dists(pose_cubs_list,pos_orient_prisos)
                if(force_on):
                    force_srv(last_attach_link,'world',pose,force,rospy.Time.from_sec(0),rospy.Duration.from_sec(-1.0))
                    force_on = False
        rate.sleep()

def VacuumService(msg):
    global vacuum_on
    if(msg.data):
        vacuum_on = True
    else:
        vacuum_on = False
    return True, 'Succsess'

if __name__=="__main__":
    rospy.init_node('vacuum_gripper')
    rospy.Subscriber("/gazebo/link_states", LinkStates, UpdateLinkStates)
    rospy.Service('/palletizer_robot/vacuum',SetBool,VacuumService)
    main()
    rospy.spin()
