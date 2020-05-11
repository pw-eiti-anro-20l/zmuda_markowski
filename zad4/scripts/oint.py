#!/usr/bin/env python

import rospy
from zad4.srv import Oint
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
import math

freq = 50    #wartosci poczatkowe
current_trans = [0, 0, 0]
current_rot = [0, 0, 0, 1]

def make_interpolation(data):
    '''
    funkcja wykonujaca obliczenia potrzebne do interpolacji ruchu ukladu kartezjanskiego
    2 sposobami: liniowym i trapezowy.
    '''
    if data.time <= 0.0:
        print "wrong value of time"
        return False
    
    x, y, z = current_trans[0], current_trans[1], current_trans[2]
    qx, qy, qz = current_rot[0], current_rot[1], current_rot[2]
    new_trans = [data.x, data.y, data.z]
    new_rot = [data.qx, data.qy, data.qz]
    rate = rospy.Rate(freq)
    current_time = 0.
    frames_number = int(math.ceil(data.time * freq))
    
    for i in range(frames_number+1):
        current_trans[0] = interpolation(x, new_trans[0], data.time, current_time, data.style)
        current_trans[1] = interpolation(y, new_trans[1], data.time, current_time, data.style)
        current_trans[2] = interpolation(z, new_trans[2], data.time, current_time, data.style)
        current_rot[0] = interpolation(qx, new_rot[0], data.time, current_time, data.style)
        current_rot[1] = interpolation(qy, new_rot[1], data.time, current_time, data.style)
        current_rot[2] = interpolation(qz, new_rot[2], data.time, current_time, data.style)
        current_time = current_time + 1.0 / freq
        rate.sleep()
    return True


#sprawdzenie rodzaju interpolacji
def interpolation(start, last, time, current_time, inter):
    '''
    funkcja powstala w celu zwikeszenia czytelnosci powtarzalnych obliczen
    sprawdza rodzaj wykonywanej interpolacji
    complex - trapezowa
    pusty lub niepoprawny argument style - liniowa
    '''
    if inter == 'complex':
        h = 2. * float(last - start) / time
        ratio = h / (time / 2.)
        if current_time < time / 2.:
            return start + current_time**2 * ratio / 2.
        else:
            return last - (time-current_time)**2 * ratio / 2.
    else: #interpolacja prostym sposobem
        return start + (float(last - start) / time) * current_time


def transformation_publisher():
    '''
    funkcja publikujaca oblicozne transformacje ukladu
    '''
    pub = rospy.Publisher('interpolation', PoseStamped, queue_size=10)
    srv = rospy.Service('oint', Oint, make_interpolation)
    rospy.init_node('oint_srv')
    print "publisher init"
    rate = rospy.Rate(freq)
    pose = PoseStamped()
    while not rospy.is_shutdown():
        pose.header.frame_id = 'base_link'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = current_trans[0]
        pose.pose.position.y = current_trans[1]
        pose.pose.position.z = current_trans[2]
        pose.pose.orientation.x = current_rot[0]
        pose.pose.orientation.y = current_rot[1]
        pose.pose.orientation.z = current_rot[2]
        pose.pose.orientation.w = 1
        pub.publish(pose)
        rate.sleep()

    print "publisher done"

if __name__ == "__main__":
    try:
        transformation_publisher()
    except rospy.ROSInterruptException:
        pass