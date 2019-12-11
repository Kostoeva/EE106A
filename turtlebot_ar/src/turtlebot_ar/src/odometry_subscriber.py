#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the dependencies as described in example_pub.py
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


#Define the callback method which is called whenever this node receives a 
#message on its subscribed topic. The received message is passed as the 
#first argument to callback().
def callback(message):

    #Print the contents of the message to the console
    print(message.pose.pose)


def listener_odom():
    rospy.init_node('listener_odom', anonymous=True)
    rospy.Subscriber("red/odom", Odometry, callback)
    rospy.spin()

#Python's syntax for a main() method
if __name__ == '__main__':
    #listener()
    listener_odom()