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
from geometry_msgs.msg import Point, Twist
from tf.transformations import euler_from_quaternion
from math import atan2
from nav_msgs.msg import Odometry

import numpy as np
import matplotlib.pyplot as plt
import math

from scipy.interpolate import interp1d

import bisect
import scipy.linalg as la

class Spline:
    """
    Cubic Spline class
    """

    def __init__(self, x, y):
        self.b, self.c, self.d, self.w = [], [], [], []

        self.x = x
        self.y = y

        self.nx = len(x)  # dimension of x
        h = np.diff(x)

        # calc coefficient c
        self.a = [iy for iy in y]

        # calc coefficient c
        A = self.__calc_A(h)
        B = self.__calc_B(h)
        self.c = np.linalg.solve(A, B)
        #  print(self.c1)

        # calc spline coefficient b and d
        for i in range(self.nx - 1):
            self.d.append((self.c[i + 1] - self.c[i]) / (3.0 * h[i]))
            tb = (self.a[i + 1] - self.a[i]) / h[i] - h[i] * \
                (self.c[i + 1] + 2.0 * self.c[i]) / 3.0
            self.b.append(tb)

    def calc(self, t):
        """
        Calc position
        if t is outside of the input x, return None
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.a[i] + self.b[i] * dx + \
            self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0

        return result

    def calcd(self, t):
        """
        Calc first derivative
        if t is outside of the input x, return None
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return result

    def calcdd(self, t):
        """
        Calc second derivative
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return result

    def __search_index(self, x):
        """
        search data segment index
        """
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        """
        calc matrix A for spline coefficient c
        """
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        #  print(A)
        return A

    def __calc_B(self, h):
        """
        calc matrix B for spline coefficient c
        """
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (self.a[i + 2] - self.a[i + 1]) / \
                h[i + 1] - 3.0 * (self.a[i + 1] - self.a[i]) / h[i]
        return B

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

def newOdom(msg):
    global x
    global y
    global theta
 
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
 
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

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
            result.extend([float(i) for i in temp])
        arr_x = result[::2]
        arr_y = result[1::2]
        print(result)
        print('\n')
        print(arr_x)
        print('\n')
        print(arr_y) 

    diff_x = arr_x[0]
    diff_y = arr_y[0]

    arr_x = np.array(arr_x)
    arr_y = np.array(arr_y)
        
    print("got here")
    x_i, y_i, yaw, k, s = calc_spline_course(arr_x - diff_x, arr_y - diff_y) 
    print("post")   

        # trajectory x, y in robot frame
    print(x_i)
    print(y_i)

    return x_i, y_i

class NavigationWaypoints():
    

    def __init__(self):

        # initiliaze
        rospy.init_node('Navigation_Waypoints', anonymous=False)

    # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")


        # What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown())
        
    # Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        #self.cmd_vel = rospy.Publisher('red/mobile_base/commands/velocity', Twist, queue_size=10)
        print("here")
        self.sub = rospy.Subscriber("/green/odom", Odometry, newOdom)
        self.pub = rospy.Publisher("/geren/mobile_base/commands/velocity", Twist, queue_size = 10)

     # get trajectory waypoints
        x_i, y_i = read_input()

        # move turtlebot to every point
       

        speed = Twist()
         
        r = rospy.Rate(4)
         
        goal = Point()
        goal.x = x_i[1]
        goal.y = y_i[1]
        print('first point',goal.x, goal.y)
         
        while not rospy.is_shutdown():
            print("start of loop")
            inc_x = goal.x -x
            inc_y = goal.y -y
         
            angle_to_goal = atan2(inc_y, inc_x)
         
            if abs(angle_to_goal - theta) > 0.1:
                speed.linear.x = 0.0
                speed.angular.z = 0.3
            else:
                speed.linear.x = 0.5
                speed.angular.z = 0.0
         
            self.pub.publish(speed)
            r.sleep() 
    #TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        #r = rospy.Rate(10);

        # Twist is a datatype for velocity
        #move_cmd = Twist()
    # let's go forward at 0.2 m/s
        #move_cmd.linear.x = 0.0
    # let's turn at 0 radians/s
        #move_cmd.angular.z = 0.2

    # as long as you haven't ctrl + c keeping doing...
        #while not rospy.is_shutdown():
        # publish the velocity
        #    self.cmd_vel.publish(move_cmd)
        # wait for 0.1 seconds (10 HZ) and publish again
        #    r.sleep()
     #current speed from turtlebot
    #


        


        # cx, cy, cyaw, ck, s = calc_spline_course(
        # ax, ay, ds=0.1)
        #ax = [0.0, 6.0, 12.5, 10.0, 17.5, 20.0, 25.0]
        #ay = [0.0, -3.0, -5.0, 6.5, 3.0, 0.0, 0.0]
        #goal = [ax[-1], ay[-1]]

        #target_speed = 10.0 / 3.6  # simulation parameter km/h -> m/s

        #sp = calc_speed_profile(yaw, target_speed)

        #t, x, y, yaw, v = do_simulation(x_i, y_i, yaw, k, sp, goal)


    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
    # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.pub.publish(Twist())
    # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)
    # lqr_Q = np.eye(5)
    # lqr_R = np.eye(2)
    # dt = 0.1  # time tick[s]
    # L = 0.5  # Wheel base of the vehicle [m]
    # max_steer = np.deg2rad(45.0)  # maximum steering angle[rad]

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


def update(state, a, delta):

    if delta >= max_steer:
        delta = max_steer
    if delta <= - max_steer:
        delta = - max_steer

    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
    state.v = state.v + a * dt

    return state


def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def solve_dare(A, B, Q, R):
    """
    solve a discrete time_Algebraic Riccati equation (DARE)
    """
    x = Q
    x_next = Q
    max_iter = 150
    eps = 0.01
    for i in range(max_iter):
        #x_next = A.T @ x @ A - A.T @ x @ B @ \
        #la.inv(R + B.T @ x @ B) @ B.T @ x @ A + Q
        # temp2 = la.inv(R + B.T @ x @ B)
        # temp = la.inv(R + np.matmul(B, np.matmul(B.T,x)))


        at_x = np.matmul(A.T, x)
        bt_x = np.matmul(B.T, x)
        at_x_b = np.matmul(at_x, B)
        bt_x_a = np.matmul(bt_x, A)
        inv_arg = R + np.matmul(bt_x, B)
        la.inv(inv_arg)

        x_next = np.matmul(at_x, A) - np.matmul(at_x_b, np.matmul(la.inv(inv_arg), bt_x_a)) + Q

        # x_next = A.T @ x @ A - A.T @ x @ B @ \
        #     la.inv(R + B.T @ x @ B) @ B.T @ x @ A + Q

        # x_next = np.matmul(np.matmul(A.T,x), A) - np.matmul(A,np.matmul(x, np.matmul(B.T, np.matmul(temp, np.matmul(B, np.matmul(A.T, x)))))) + Q
        if (abs(x_next - x)).max() < eps:
            break
        x = x_next

    return x_next


def dlqr(A, B, Q, R):
    """Solve the discrete time lqr controller.
    x[k+1] = A x[k] + B u[k]
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    """

    # first, try to solve the ricatti equation
    X = solve_dare(A, B, Q, R)

    # compute the LQR gain
    bt_x = np.matmul(B.T, X)
    bt_x_b = np.matmul(bt_x, B)
    bt_x_a = np.matmul(bt_x, A)
    K = np.matmul(la.inv(bt_x_b + R), bt_x_a)
    # K = la.inv(B.T @ X @ B + R) @ (B.T @ X @ A)

    # K = np.matmul(la.inv(np.matmul(B,np.matmul(B.T, X))), np.matmul(A, np.matmul(B.T, X)))

    #eig_result = la.eig(A - B @ K)
    eig_result = la.eig(A - np.matmul(B,K))

    return K, X, eig_result[0]


def lqr_speed_steering_control(state, cx, cy, cyaw, ck, pe, pth_e, sp, Q, R):
    ind, e = calc_nearest_index(state, cx, cy, cyaw)

    tv = sp[ind]

    k = ck[ind]
    v = state.v
    th_e = pi_2_pi(state.yaw - cyaw[ind])

    # A = [1.0, dt, 0.0, 0.0, 0.0
    #      0.0, 0.0, v, 0.0, 0.0]
    #      0.0, 0.0, 1.0, dt, 0.0]
    #      0.0, 0.0, 0.0, 0.0, 0.0]
    #      0.0, 0.0, 0.0, 0.0, 1.0]
    A = np.zeros((5, 5))
    A[0, 0] = 1.0
    A[0, 1] = dt
    A[1, 2] = v
    A[2, 2] = 1.0
    A[2, 3] = dt
    A[4, 4] = 1.0

    # B = [0.0, 0.0
    #     0.0, 0.0
    #     0.0, 0.0
    #     v/L, 0.0
    #     0.0, dt]
    B = np.zeros((5, 2))
    B[3, 0] = v / L
    B[4, 1] = dt

    K, _, _ = dlqr(A, B, Q, R)

    # state vector
    # x = [e, dot_e, th_e, dot_th_e, delta_v]
    # e: lateral distance to the path
    # dot_e: derivative of e
    # th_e: angle difference to the path
    # dot_th_e: derivative of th_e
    # delta_v: difference between current speed and target speed
    x = np.zeros((5, 1))
    x[0, 0] = e
    x[1, 0] = (e - pe) / dt
    x[2, 0] = th_e
    x[3, 0] = (th_e - pth_e) / dt
    x[4, 0] = v - tv

    # input vector
    # u = [delta, accel]
    # delta: steering angle
    # accel: acceleration
    #ustar = -K @ x
    ustar = np.matmul(-K, x)

    # calc steering input
    ff = math.atan2(L * k, 1)  # feedforward steering angle
    fb = pi_2_pi(ustar[0, 0])  # feedback steering angle
    delta = ff + fb

    # calc accel input
    accel = ustar[1, 0]

    return delta, ind, e, th_e, accel


def calc_nearest_index(state, cx, cy, cyaw):
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind)

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind

def do_simulation(cx, cy, cyaw, ck, speed_profile, goal):
    T = 500.0  # max simulation time
    goal_dis = 0.3
    stop_speed = 0.05

    state = State(x=-0.0, y=-0.0, yaw=0.0, v=0.0)

    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]

    e, e_th = 0.0, 0.0

    while T >= time:
        dl, target_ind, e, e_th, ai = lqr_speed_steering_control(
            state, cx, cy, cyaw, ck, e, e_th, speed_profile, lqr_Q, lqr_R)

        state = update(state, ai, dl)

        if abs(state.v) <= stop_speed:
            target_ind += 1

        time = time + dt

        # check goal
        dx = state.x - goal[0]
        dy = state.y - goal[1]
        if math.hypot(dx, dy) <= goal_dis:
            print("Goal")
            break

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
    return t, x, y, yaw, v

def calc_speed_profile(cyaw, target_speed):
    speed_profile = [target_speed] * len(cyaw)

    direction = 1.0

    # Set stop point
    for i in range(len(cyaw) - 1):
        dyaw = abs(cyaw[i + 1] - cyaw[i])
        switch = math.pi / 4.0 <= dyaw < math.pi / 2.0

        if switch:
            direction *= -1

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

        if switch:
            speed_profile[i] = 0.0

    # speed down
    for i in range(40):
        speed_profile[-i] = target_speed / (50 - i)
        if speed_profile[-i] <= 1.0 / 3.6:
            speed_profile[-i] = 1.0 / 3.6

    return speed_profile


 
if __name__ == '__main__':
    try:
        print('pre-test')

        NavigationWaypoints()
        
    except:
        rospy.loginfo("NavigationWaypoints node terminated.")