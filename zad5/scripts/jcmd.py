#!/usr/bin/env python

import sys
import rospy
from zad4.srv import Interpolation

def interpolation_client(x, y, z, time, style):
    rospy.wait_for_service('int')
    try:
        interpolation = rospy.ServiceProxy('int', Interpolation)
        response = interpolation(x, y, z, time, style)
        return response.status
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    #wczytanie danych interpolacji ukladu
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    z = float(sys.argv[3])
    time = float(sys.argv[4])
    style = sys.argv[5]
    print "Requesting interpolation"
    #rozpoczecie serwisu interpolacji ukladu
    print interpolation_client(x, y, z, time, style)