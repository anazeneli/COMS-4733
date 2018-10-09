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
    global g_range_left
    
    g_range_ahead = msg.ranges[len(msg.ranges)/2]
    g_range_right = msg.ranges[0]
    g_range_left  = msg.ranges[len(msg.ranges) -1]

g_range_ahead = 1 # initialize globals
g_range_right = 1
g_range_left  = 1

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

def hit_point_check(x,y):
    global hit_count
    for i, j in hits:
        if x - i  < -0.2 and abs(j - y) < 0.2:
            return True

    return False

class bug2():
    def __init__(self):
        scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
        hit_count = 0
        
        while not rospy.is_shutdown():
            update_odom()
            print "angle", angle_to_goal
            print g_range_left, g_range_ahead, g_range_right
            if g_range_ahead < 3:
                print "Approaching Obstacle"
                
            if g_range_ahead < 0.8:
                t.stopRobot()
                print "OBSTACLE"
                
                if g_range_ahead < 0.65:
                    t.translate(-0.2)
                
                # STORE HIT POINT
                update_odom()
                hits.append([x,y])
                print hits
                    
                # TURN LEFT
                print "Turning LEFT",g_range_left,  g_range_right, g_range_ahead
                if isnan(g_range_right) :
                    while not isnan(g_range_ahead):
                        print g_range_right, g_range_ahead

                        if g_range_ahead > 1 :
                            break
                        t.rotate(20)
                    t.translate(0.25)
                else:
                    print "H"
                    # until nothing detected on right w/in 3m
                    while  not isnan(g_range_right) :
                        print g_range_right, g_range_ahead
                        if g_range_ahead > 1.2 :
                            break
                        t.rotate(15)

                print "follow obstacle", g_range_left, g_range_right, g_range_ahead

                t.translate(1)
                update_odom()

                while t.on_mline(y) == False or hit_point_check(x,y) == False:
                    print "Navigating "
                    print g_range_left, g_range_ahead, g_range_right

                    if g_range_ahead < 0.8:
                        t.translate(-0.2)

                    t.translate(0.1)

                    if g_range_right < 0.8:
                        t.rotate(15)
                        t.translate(.15)
                        
                    t.translate(0.1)
                    
                    if isnan(g_range_right) or g_range_right > 3:
                        t.rotate(-30)
                        t.rotate(30)
                        
                    update_odom()
                    print x , y, hit_point_check(x,y)

                    # if on the m-line higher up, rotate
                    if t.on_mline(y):
                        print "On m-line again "
                        if abs(x_hit -x ) > 0.5 :
                            t.stopRobot()
                            dif = angle_to_goal - theta
                            if abs(dif) > 0.1:
                                t.rotate(degrees(dif))
                        break
                        
                if  hit_point_check(x,y) :
                    print "HIT"
                    while isnan(g_range_right):
                        t.rotate(-15)
                        print g_range_left, g_range_ahead, g_range_right
                   
                t.translate(1.5)
                            
            elif abs(goal.x - x ) < 0.15:
                t.stopRobot()
                dif = angle_to_goal - theta
                if abs(dif) > 0.1:
                    t.rotate(degrees(dif))
                t.shutdown()    
            else:
                if  hit_point_check(x,y):
                    print "HIT"
                    hit_count +=1

                    if hit_count >= 1:
                        print "IMPOSSIBLE TO REACH GOAL"
                        t.stopRobot()
                        t.shutdown()

                # go one meter at a time until obstacle found
                # or if off m-line, readjust
                print g_range_ahead
                dif = angle_to_goal - theta
                if abs(dif ) > 0.1:
                    t.rotate(degrees(dif))

                t.translate(0.25)



if __name__ == '__main__':
    try:
        t = TurtlebotController()
    except:
        rospy.loginfo("TurtlebotController node terminated.")
    bug2()
