#!/usr/bin/env python

import rospy
import json
import os
import PyKDL as kdl
import math
from tf.transformations import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

#sprawdzenie poprawnosci polozenia dla kazdego zlacza
def checkPosition(data):
    if data.position[0] < bounds['i1'][0] or data.position[0] > bounds['i1'][1]:
        return False
    if data.position[1] < bounds['i2'][0] or data.position[1] > bounds['i2'][1]:
        return False
    if data.position[2] < bounds['i3'][0] or data.position[2] > bounds['i3'][1]:
        return False
    return True

def simpleKinematics(data):
    #jesli polozenie jest niepoprawne zostaje wypisana informacja
    if not checkPosition(data):
        rospy.logerr('Wrong position: ' + str(data))
        return

    #utworzenie trzech kolejnych macierzy z parametrow D-H
    kdl_chain = kdl.Chain()
    kdl_frame = kdl.Frame()

    frame0 = kdl_frame.DH(0, 0, 0, 0)
    joint0 = kdl.Joint(kdl.Joint.None)
    kdl_chain.addSegment(kdl.Segment(joint0, frame0))

    a, d, alpha, theta = params['i2']
    frame1 = kdl_frame.DH(a, alpha, d, theta)
    joint1 = kdl.Joint(kdl.Joint.RotZ)
    kdl_chain.addSegment(kdl.Segment(joint1, frame1))

    a, d, alpha, theta = params['i1']
    frame2 = kdl_frame.DH(a, alpha, d, theta)
    joint2 = kdl.Joint(kdl.Joint.RotZ)
    kdl_chain.addSegment(kdl.Segment(joint2, frame2))

    a, d, alpha, theta = params['i3']
    frame3 = kdl_frame.DH(a, alpha, d, theta)
    joint3 = kdl.Joint(kdl.Joint.TransZ)
    kdl_chain.addSegment(kdl.Segment(joint3, frame3))

    #przeliczenie poprzednich macierzy na finalne wartosci koncowki
    joints_angle = kdl.JntArray(kdl_chain.getNrOfJoints())
    joints_angle[0] = data.position[0]
    joints_angle[1] = data.position[1]
    joints_angle[2] = -data.position[2]

    kdl_chain_solver = kdl.ChainFkSolverPos_recursive(kdl_chain)
    final_frame = kdl.Frame()
    kdl_chain_solver.JntToCart(joints_angle, final_frame)
    quaternion = final_frame.M.GetQuaternion()

    #utworzenie i nadanie wiadomosci z polozeniem koncowki
    pose = PoseStamped()
    pose.header.frame_id = 'base'
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = final_frame.p[0]
    pose.pose.position.y = final_frame.p[1]
    pose.pose.position.z = final_frame.p[2]
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]
    pub.publish(pose)

if __name__ == '__main__':
    #utworzenie wezla
    rospy.init_node('KDL_DKIN', anonymous=True)
    pub = rospy.Publisher('kdl_msgs', PoseStamped, queue_size=10)
    rospy.Subscriber('joint_states', JointState, simpleKinematics)

    #pobranie parametrow i ograniczen
    params = {}
    with open(os.path.dirname(os.path.realpath(__file__)) + '/../MDH_wartosci.json', 'r') as file:
        params = json.loads(file.read())

    bounds = {}
    with open(os.path.dirname(os.path.realpath(__file__)) + '/../bounds.json', 'r') as file:
        bounds = json.loads(file.read())

    rospy.spin()