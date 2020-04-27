#!/usr/bin/env python

import rospy
import json
import PyKDL as kdl
import math
import os
from tf.transformations import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

def find_position(data):
    '''
    spr czy pozycja nie jest bledna, tj. po za ograniczeniami spowodowanymi budowa robota
    limity wczytane z przygotowanego pliku
    '''
    if data.position[0] < limit['i1'][0] or data.position[0] > limit['i1'][1]:
        return False
    if data.position[1] < limit['i2'][0] or data.position[1] > limit['i2'][1]:
        return False
    if data.position[2] < limit['i3'][0] or data.position[2] > limit['i3'][1]:
        return False
    return True

def simple_kinematic(data):
    '''
    realizacja kinematyki prostej z uzyciem biblioteki KDL
    wraz z nadaniem obliczonych wartosci
    '''
    if not find_position(data):
        rospy.logerr('Wrong position: ' + str(data)) #spr czy nie bledna pozycja
        return

    kdl_chain = kdl.Chain()
    kdl_frame = kdl.Frame()

    frame0 = kdl_frame.DH(0, 0, 0, 0)
    joint0 = kdl.Joint(kdl.Joint.None)
    kdl_chain.addSegment(kdl.Segment(joint0, frame0))

    a, d, alfa, theta = params['i2']
    frame1 = kdl_frame.DH(a, alfa, d, theta)
    joint1 = kdl.Joint(kdl.Joint.RotZ)
    kdl_chain.addSegment(kdl.Segment(joint1, frame1))

    a, d, alfa, theta = params['i1']
    frame2 = kdl_frame.DH(a, alfa, d, theta)
    joint2 = kdl.Joint(kdl.Joint.RotZ)
    kdl_chain.addSegment(kdl.Segment(joint2, frame2))

    a, d, alfa, theta = params['i3']
    frame3 = kdl_frame.DH(a, alfa, d, theta)
    joint3 = kdl.Joint(kdl.Joint.TransZ)
    kdl_chain.addSegment(kdl.Segment(joint3, frame3))

    joint_angles = kdl.JntArray(kdl_chain.getNrOfJoints())
    joint_angles[0] = data.position[0]
    joint_angles[1] = data.position[1]
    joint_angles[2] = -data.position[2]
    
    #kinematyka prosta przeliczenie na translacje i rotacje czlonu koncowego
    kdl_chain_t = kdl.ChainFkSolverPos_recursive(kdl_chain)
    frame_t = kdl.Frame()
    kdl_chain_t.JntToCart(joint_angles, frame_t)
    quat = frame_t.M.GetQuaternion()

    pose = PoseStamped()
    pose.header.frame_id = 'base_link'
    pose.header.stamp = rospy.Time.now()
    #nadanie wiadomosci z wyliczonymi wspolrzednymi czlonu koncowego
    pose.pose.position.x = frame_t.p[0]
    pose.pose.position.y = frame_t.p[1]
    pose.pose.position.z = frame_t.p[2]
    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]
    pub.publish(pose)

if __name__ == '__main__':
    rospy.init_node('KDL_DKIN', anonymous=True)
    pub = rospy.Publisher('kdl_msgs', PoseStamped, queue_size=10)
    rospy.Subscriber('joint_states', JointState, simple_kinematic)

    params = {}
    limit = {}

    with open(os.path.dirname(os.path.realpath(__file__)) + '/../dh.json', 'r') as file:
        params = json.loads(file.read())

    with open(os.path.dirname(os.path.realpath(__file__)) + '/../limit.json', 'r') as file:
        limit = json.loads(file.read())

    rospy.spin()
