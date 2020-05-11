#!/usr/bin/env python

import rospy
import time
from zad4.srv import Interpolation
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

freq = 50    #wartosci poczatkowe
current_pos = [0,0,0]


def make_interpolation(data):
    '''
    funkcja wykonujaca obliczenia potrzebne do interpolacji ruchu robota
    2 sposobami: liniowym i trapezowy.
    poczatkowo funkcja sprawdza czy wartosci parametrow przekazanych do 
    programu nie przekraczaja ustawionych limitow robota
    '''
    if data.time <= 0 or not -1.57075 <= data.j1 <= 1.57075 or not -1.57075 <= data.j2 <= 1.57075 or not 0 <= data.j3 <= 0.2:
        #do modyfikacji!!! zrobic petle sprawdzajaca dane aby wyswietlic
        #uzytkownikowi jaki argument wprowadzil bledny
        print "wrong data"
        return False

    new_pos = [data.j1, data.j2, data.j3]
    rate = rospy.Rate(freq)
    j1, j2, j3 = current_pos[0], current_pos[1], current_pos[2]
    frames_number = int(math.ceil(data.time * freq))
    current_time = 0.

    for k in range(0, frames_number + 1):
        current_pos[0] = interpolation(j1, new_pos[0], data.time, current_time, data.style)
        current_pos[1] = interpolation(j2, new_pos[1], data.time, current_time, data.style)
        current_pos[2] = interpolation(j3, new_pos[2], data.time, current_time, data.style)
        current_time = current_time + 1. / freq
        rate.sleep()

    return True

def interpolation(start, last, time, current_time, inter):
    if inter == 'complex':
        h = 2. * float(last - start) / time
        ratio = h / (time / 2.)
        if current_time < time / 2.:
            return start + current_time**2 * ratio / 2.
        else:
            return last - (time-current_time)**2 * ratio / 2.
    else: #interpolacja prostym sposobem
        return start + (float(last - start) / time) * current_time


def joint_publisher():
    '''
    funkcja publikujaca oblicozne transformacje ukladu
    '''
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    srv = rospy.Service('int', Interpolation, make_interpolation)
    rospy.init_node('int_srv')
    rate = rospy.Rate(100)
    joint_state = JointState()
    while not rospy.is_shutdown():
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['base_to_link1', 'link1_to_link2', 'link2_to_link3']
        joint_state.position = current_pos
        joint_state.velocity = [0,0,0]
        joint_state.effort = [0,0,0]
        pub.publish(joint_state)

if __name__ == "__main__":
    try:
        joint_publisher()
    except rospy.ROSInterruptException:
        pass