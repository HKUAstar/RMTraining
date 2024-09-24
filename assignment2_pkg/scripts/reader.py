#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from time import sleep

def callback(msg):
     global status
     status = msg.data
def publisher():
    # First, create objects
    rospy.init_node("read_inputs",anonymous=True)
    pub1 = rospy.Publisher("input_1", Float64, queue_size= 10)
    # queue_size = 10 defines how many messages can be buffered
    # with queue_size = 0, up tp 10 messages can be stored in the publisher, if publisher works faster than subscribers can process them.
    pub2 = rospy.Publisher("input_2", Float64, queue_size= 10)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        try:
            rospy.sleep(0.5)
            input1 = float(input("Enter a number: "))
            input2 = float(input("Enter another number: "))
            pub1.publish(input1)
            pub2.publish(input2)
            rospy.loginfo("Two inputs has been published to input_1, input_2.")
        except ValueError:
            rospy.loginfo("Invalid input value, publishing denied.")

if __name__ == "__main__":
        try:
            publisher()
        except rospy.ROSInterruptException:
             pass