#!/usr/bin/env python3
"""
This scripts need to subscribe the topic /scan
and interpret the data to 2D coordinates
and save the pairs to points.txt (perhaps do so in the question2.launch?)
"""


import rospy
from sensor_msgs.msg import LaserScan
import math

points = [] # use a list of tuples to store points

def callback(scan):
    global points
    points.clear()
    for i in range(len(scan.ranges)):
        angle = scan.angle_min + i*scan.angle_increment
        distance = scan.ranges[i]
        if scan.range_min < distance < scan.range_max:
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            points.append((x,y))
    print("-------------------------------")
    for x, y in points:
        print(f"({x}, {y})")

if __name__ == '__main__':
    rospy.init_node('points', anonymous = True) #change the name from to_points to points, but it is of no use (perhaps)
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    rospy.loginfo("Listening to /scan topic ...")
    rospy.spin()