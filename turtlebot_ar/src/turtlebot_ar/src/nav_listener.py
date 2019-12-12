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

class cycle_listener:
    def __init__(self):

        #Run this program as a new node in the ROS computation graph
        #called /listener_<id>, where <id> is a randomly generated numeric
        #string. This randomly generated name means we can start multiple
        #copies of this node without having multiple nodes with the same
        #name, which ROS doesn't allow.
        rospy.init_node('cycle_listener', anonymous=True)

        #Create a new instance of the rospy.Subscriber object which we can 
        #use to receive messages of type std_msgs/String from the topic /chatter_talk.
        #Whenever a new message is received, the method callback() will be called
        #with the received message as its first argument.
        self.sub = rospy.Subscriber("/green/mobile_base/commands/velocity", Twist, self.callback)
        self.pub = rospy.Publisher("/green/navigation", Twist, queue_size=100)
        rospy.spin()

    def callback(self,message):
        twist = message
        self.pub.publish(twist)


#Python's syntax for a main() method
if __name__ == '__main__':
    #listener()
    cycle_listener()