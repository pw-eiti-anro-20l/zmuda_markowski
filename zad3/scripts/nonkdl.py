#!/usr/bin/env python

import rospy
import json
from tf.transformations import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


def find_position(data):
'''
spr czy pozycja nie jest błędna, tj. po za ograniczeniami spowodowanymi budową robota
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
realizacja kinematyki prostej bez uzycia biblioteki KDL
wraz z nadaniem obliczonych wartosci
'''
    if not find_position(data):
        rospy.logerr('Wrong position: ' + str(data)) #spr czy nie błędna pozycja
        return

    x, z = (1, 0, 0), (0, 0, 1) # wektory dla osi w notacji dh potrzebne do macierzy rotacji

    a, d, alfa, theta = params['i1']
    trans_z = translation_matrix((0, 0, d))
    rot_z = rotation_matrix(data.position[0], z)
    trans_x = translation_matrix((a, 0, 0))
    rot_x = rotation_matrix(alfa, x)
    matrix_1 = concatenate_matrices(rot_x, trans_x, rot_z, trans_z) #macierz_1

    a, d, alfa, theta = params['i2']
    trans_z = translation_matrix((0, 0, d))
    rot_z = rotation_matrix(data.position[1], z)
    trans_x = translation_matrix((a, 0, 0))
    rot_x = rotation_matrix(alfa, x)
    matrix_2 = concatenate_matrices(rot_x, trans_x, rot_z, trans_z) #macierz_2

    a, d, alfa, theta = params['i3']
    trans_z = translation_matrix((0, 0, data.position[2]-d))
    rot_z = rotation_matrix(theta, z)
    trans_x = translation_matrix((a, 0, 0))
    rot_x = rotation_matrix(alfa, x)
    matrix_3 = concatenate_matrices(rot_x, trans_x, rot_z, trans_z) #macierz_3


    #kinematyka prosta przeliczenie na translacje i rotacje czlonu koncowego
    matrix_t = concatenate_matrices(matrix_1, matrix_2, matrix_3)
    x_t, y_t, z_t = translation_from_matrix(matrix_t)
    quat_x, quat_y, quat_z, quat_w = quaternion_from_matrix(matrix_t)

    pose = PoseStamped()
    pose.header.frame_id = 'base_link'
    pose.header.stamp = rospy.Time.now()
    #nadanie wiadomosci z wyliczonymi wspolrzednymi czlonu koncowego
    pose.pose.position.x = x_t
    pose.pose.position.y = y_t
    pose.pose.position.z = z_t
    pose.pose.orientation.x = quat_x
    pose.pose.orientation.y = quat_y
    pose.pose.orientation.z = quat_z
    pose.pose.orientation.w = quat_w
    pub.publish(pose)

if __name__ == '__main__':
    rospy.init_node('NONKDL_DKIN', anonymous=True)
    pub = rospy.Publisher('non_kdl_msgs', PoseStamped, queue_size=10)
    rospy.Subscriber('joint_states', JointState, simple_kinematic)

    params = {}
    limit = {}

    with open('../dh.json', 'r') as file:
        params = json.loads(file.read())

    with open('../limit.json', 'r') as file:
        limit = json.loads(file.read())

    rospy.spin()

