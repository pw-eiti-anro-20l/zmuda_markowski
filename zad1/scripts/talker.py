#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from curtsies import Input


param_name = rospy.search_param("sim/")
keys = rospy.get_param(param_name)

def get_key():
    with Input(keynames='curses') as in_gen:
        for e in in_gen:
            if e in keys.values():
                key = e
                break
    return key


def talker():     
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)    
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100)
    twist = Twist()

    while not rospy.is_shutdown():
        press = get_key()     
        
        if press == keys["forward"]:
            print("do przodu")
            twist.linear.x = 2
            twist.angular.z = 0
        elif press == keys["back"]:
            print("do tylu")
            twist.linear.x = -2
            twist.angular.z = 0
        elif press == keys["left"]:
            print("lewo")
            twist.angular.z = 2
            twist.linear.x = 0
        elif press == keys["right"]:
            print("prawo")
            twist.angular.z = -2
            twist.linear.x = 0
        else:
            twist.linear.x = 0
            twist.angular.z = 0
        pub.publish(twist)


if __name__ == '__main__':
    try:
        print("start")
        talker()
    except rospy.ROSInterruptException as e:
        print e