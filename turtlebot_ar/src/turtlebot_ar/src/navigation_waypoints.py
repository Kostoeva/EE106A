#!/usr/bin/env python

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# A very basic TurtleBot script that moves TurtleBot forward indefinitely. Press CTRL + C to stop.  To run:
# On TurtleBot:
# roslaunch turtlebot_bringup minimal.launch
# On work station:
# python navigation_waypoints.py
import rospy
import sys
import os
from geometry_msgs.msg import Twist



import numpy as np
import matplotlib.pyplot as plt
import math

from scipy.interpolate import interp1d


import bisect
import scipy.linalg as la



class Spline2D:
    """
    2D Cubic Spline class
    """

    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.sx = Spline(self.s, x)
        self.sy = Spline(self.s, y)

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = np.hypot(dx, dy)
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        """
        calc position
        """
        x = self.sx.calc(s)
        y = self.sy.calc(s)

        return x, y

    def calc_curvature(self, s):
        """
        calc curvature
        """
        dx = self.sx.calcd(s)
        ddx = self.sx.calcdd(s)
        dy = self.sy.calcd(s)
        ddy = self.sy.calcdd(s)
        k = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(3 / 2))
        return k

    def calc_yaw(self, s):
        """
        calc yaw
        """
        dx = self.sx.calcd(s)
        dy = self.sy.calcd(s)
        yaw = math.atan2(dy, dx)
        return yaw


    def calc_spline_course(x, y, ds=0.1):
        sp = Spline2D(x, y)
        s = list(np.arange(0, sp.s[-1], ds))

        rx, ry, ryaw, rk = [], [], [], []
        for i_s in s:
            ix, iy = sp.calc_position(i_s)
            rx.append(ix)
            ry.append(iy)
            ryaw.append(sp.calc_yaw(i_s))
            rk.append(sp.calc_curvature(i_s))

        return rx, ry, ryaw, rk, s



class NavigationWaypoints():

    def __init__(self):
        # initiliaze
        rospy.init_node('Navigation_Waypoints', anonymous=False)

    # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")

        # What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown)
        
    # Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        self.cmd_vel = rospy.Publisher('red/mobile_base/commands/velocity', Twist, queue_size=10)
     
    #TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        r = rospy.Rate(10);

        # Twist is a datatype for velocity
        move_cmd = Twist()
    # let's go forward at 0.2 m/s
        move_cmd.linear.x = 0.2
    # let's turn at 0 radians/s
        move_cmd.angular.z = 0.2

    # as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
        # publish the velocity
            self.cmd_vel.publish(move_cmd)
        # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()
     #current speed from turtlebot
    #
    def read_input(self):
        print('test')
        filename = sys.argv[1]
        with open(filename) as f:
            content = f.read()
            print(content)
            result = []
            arr_x, arr_y = [], []
            temp = content.split('\n')
            for line in temp:  
                temp = line.split(',')
                result.extend(temp)
            arr_x = result[::2]
            arr_y = result[1::2]
            print(result)
            print('\n')
            print(arr_x)
            print('\n')
            print(arr_y) 

        diff_x = arr_x[0]
        diff_y = arr_y[0]
        print("here0")


        spline = Spline2D()
        x_i, y_i, yaw, k, s = spline.calc_spline_course(arr_x[1:] + diff_x, arr_y[1:] + diff_y, ds =0.1) 
        print("post")   

        print(x_i)
        print(y_i)


    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
    # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
    # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)

 
if __name__ == '__main__':
    try:
        print('pre-test')
        NavigationWaypoints().read_input()
        
    except:
        rospy.loginfo("NavigationWaypoints node terminated.")