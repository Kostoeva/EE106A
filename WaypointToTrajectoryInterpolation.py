#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np
import matplotlib.pyplot as plt
import math


# In[29]:


error = 0.1
x0 = 1
y0 = 1
r0 = 0.5

obstacles = [(0.06, 0.045)]

def get_points_around_ob(radius, ob_center, start_pt, end_pt):
    path = []
    d = 10
    intervals = math.pi / d
    angle = 0
    for i in range(d):
        angle+=intervals
        path.append(((ob_center[0]+(radius*np.cos(angle))),(ob_center[1]+(radius*np.sin(angle)))))
    
    return path    
   
for o in obstacles :
    (get_points_around_ob(0.02, (0.05, 0.07), (0.02, 0.07), (0.08, 0.07)))
#insert intermediate waypoints from navigating around the obstacle

#slicing 

x = [0.01, 0.02, 0.03, 0.06, 0.07]
y = [0.02, 0.05, 0.07, 0.04, 0.02]

#inputted waypts
plt.scatter(x, y, color='blue', label='given')

'''
if an obstacle exists on the interpolated path within some 
error margin [RADIUS OF OBJECT], route around it within those 2 way pts, use them as start and end
'''


# In[35]:


from scipy.interpolate import *

from scipy.interpolate import CubicSpline

for i in range(0, len(x)-1):
    cs = CubicSpline(x[i:i+2], y[i:i+2])
    #plt.plot(x[i:i+2], cs(x[i:i+2]), label="S")
    print((x[i:i+2], cs(x[i:i+2])))
    plt.show()


# In[ ]:




