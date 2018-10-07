#!/usr/bin/env python

import rospy
from turtlebot_controller import *
t = TurtlebotController()
from sensor_msgs.msg import LaserScan
from math import pi, isnan, atan2, degrees

hits = []

def scan_callback(msg):
    global g_range_ahead
    global g_range_right
    global g_range_min


    g_range_ahead = msg.ranges[len(msg.ranges)/2]
    g_range_right = msg.ranges[0]
    g_range_min  = min(msg.ranges)

g_range_ahead = 1 # anything to start
g_range_right = 1
g_range_min  = 1

(position, rotation) = t.get_odom()
start_theta = rotation

def update_odom():
    global x, y , theta, angle_to_goal

    (position, rotation) = t.get_odom()
    x = position.x
    y = position.y
    theta = rotation

    inc_x = goal.x - x
    inc_y = goal.y - y
    angle_to_goal = atan2(inc_y, inc_x)

def reached_hit_point(x,y):
    for i, j in hits:
        if i -x < 0.1 and j - y < 0.1:
            return True

    return False

class bug2():
    def __init__(self):
        scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)

        while not rospy.is_shutdown():
            update_odom()
            print "angle", angle_to_goal
            print g_range_ahead, g_range_right

            if g_range_ahead < 0.9:
                t.stopRobot()
                print "OBSTACLE"
                # STORE HIT POINT
                (position, rotation) = t.get_odom()
                x_hit = x + g_range_ahead
                y_hit = position.y
                hits.append([x_hit, y_hit])
                print hits
                if g_range_min < 0.8:
                    print "CLOSE", g_range_min
                    t.rotate(10)

                # TURN LEFT
                # until nothing detected on right w/in 3m
                while not isnan(g_range_right) or g_range_right > 3:
                    print "TURN LEFT"
                    t.rotate(15)

                t.translate(1)
                update_odom()

                while t.on_mline(y) == False :
                    print "Navigating", g_range_ahead, g_range_right
                    t.translate(0.1)

                    if isnan(g_range_right):
                        t.rotate(-30)

                    if g_range_right < 1.6:
                        t.rotate(15)
                        t.translate(.25)

                    update_odom()

                # once on m-line again, readjust toward goal
                if t.on_mline(y):
                    dif = angle_to_goal - theta
                    if abs(dif ) > 0.1:
                        t.rotate(degrees(dif))
                t.stopRobot()
            else:
                # go one meter at a time until obstacle found
                # or if off m-line, readjust
                print g_range_ahead
                dif = angle_to_goal - theta
                if abs(dif ) > 0.1:
                    t.rotate(degrees(dif))

                t.translate(1)



if __name__ == '__main__':
    try:
        t = TurtlebotController()
    except:
        rospy.loginfo("TurtlebotController node terminated.")
    bug2()
