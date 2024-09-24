#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64, Bool

input1 = 0.0
input2 = 0.0
def callback1(data):
    global input1
    input1 = data.data
    rospy.loginfo("Received input1. Saved input1 to input1 (global variable).")

def callback2(data):
    global input2, input1
    input2 = data.data
    rospy.loginfo("Received input2. Saved input1 to input1 (global variable).")
    print(f"The sum of two current inputs: {input1+ input2}")

def sum_subscriber():
    global input1, input2
    rospy.init_node("sum_subscriber", anonymous= True)
    sub1 = rospy.Subscriber("input_1", Float64, callback1)
    sub2 = rospy.Subscriber("input_2", Float64, callback2)
    rospy.spin() # Run the node(!), instead of statements in the function once received a message

if __name__ == "__main__":
    sum_subscriber()