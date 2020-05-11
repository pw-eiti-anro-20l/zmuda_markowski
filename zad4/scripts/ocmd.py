#!/usr/bin/env python

import sys
import rospy
from zad4.srv import Oint

#funkcja rzadajaca interpolacji ukladu
def interpolation_client(x, y, z, qx, qy, qz, time, style):
    rospy.wait_for_service('oint')
    try:
        interpolation = rospy.ServiceProxy('oint', Oint)
        response = interpolation(x, y, z, qx, qy, qz, time, style)
        return response.status
    except rospy.ServiceException, e:
        print "Service failed: %s"%e

if __name__ == "__main__":
    #wczytanie danych interpolacji ukladu
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    z = float(sys.argv[3])
    qx = float(sys.argv[4])
    qy = float(sys.argv[5])
    qz = float(sys.argv[6])
    time = float(sys.argv[7])
    style = sys.argv[8]
    print "Requesting interpolation."
    #rozpoczecie serwisu interpolacji ukladu
    print interpolation_client(x, y, z, qx, qy, qz, time, style)