# Testing algorithm to estimate distance to source
#
# By Mark Watson
# May 10, 2022

#!/usr/bin/env python3

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as gl
import numpy as np
import random as r
from packages.receiver import Receiver

# Computes the nearest points on two skew lines. 
# The nearest point is the intersection of line1 with a plane that includes line2 and a normal vector to lines 1 and 2.
def compute_nearest_point(input):

    dp = np.dot(input[2], input[3])

    if (dp == 0.0): # check divide by zero
        return [max_dist, max_dist, max_dist]

    return (input[0] + input[2]*np.dot((input[1]-input[0]), input[3])/dp)

# Plots unit vectors that start from a receiver and point in direction of the source signal
def plot_receiver_vectors(input):

    for row in input:
        sp = gl.GLLinePlotItem(pos=np.array([row[0]+row[1], row[1]]), color=row[2], width=5.0)
        w.addItem(sp)

# Computes normal vectors that are normal to lines along v1, v2, and v3
def get_normal_vectors(v1, v2, v3):

    # vectors that are normal to 1 and 2
    n12 = np.cross(v1, v2)
    n21 = np.cross(v2, n12)
    # vectors that are normal to 1 and 3
    n13 = np.cross(v1, v3)
    n31 = np.cross(v3, n13)
    # vectors that are normal to 2 and 3
    n23 = np.cross(v2, v3)
    n32 = np.cross(v3, n23)

    return [n12, n21, n13, n31, n23, n32]

# Gets all vectors needed for calculations
def get_all_vectors():

    # /todo: using random variation for now, but will read from serial port in future
    inc = np.deg2rad(variation*(r.random()-0.5)+90)
    azm = np.deg2rad(variation*(r.random()-0.5)+90)

    [p1, v1] = rx1.compute_vectors(inc, azm-np.deg2rad(variation/4))
    [p2, v2] = rx2.compute_vectors(inc-np.deg2rad(variation/6), azm)
    [p3, v3] = rx3.compute_vectors(inc+np.deg2rad(variation/8), azm)

    [n12, n21, n13, n31, n23, n32] = get_normal_vectors(v1, v2, v3)
    
    receivers = [[v1, p1, red],
                 [v2, p2, blue],
                 [v3, p3, green]]

    plot_receiver_vectors(receivers)

    vectors = [[p1, p2, v1, n21],
               [p2, p1, v2, n12],
               [p1, p3, v1, n31],
               [p3, p1, v3, n13],
               [p2, p3, v2, n32],
               [p3, p2, v3, n23]]

    return vectors

# Calculates the average distance from a receiver to the nearest points, as described above
def compute_average_distance(distances):

    length = len(distances)

    if length == 0: # check for divide by zero
        print("avg dist: undefined")
    else:
        avg = sum(distances)/length
        print("avg dist: " + str(round(avg,2)) + " ft")

# Main algorithm for determining nearest points then estimating distance to source
def run_algorithm():

    w.clear()
    w.addItem(g)

    vectors = get_all_vectors()
    distances = []

    for v in vectors:
        n_pt = compute_nearest_point(v) # n_pt := nearest point
        if (np.linalg.norm(n_pt) < max_dist) and (np.linalg.norm(n_pt) > min_dist) and (n_pt[1] > 0):
            sp = gl.GLScatterPlotItem(pos=n_pt, size=0.5, color=white, pxMode=False)
            w.addItem(sp)
            distances.append(np.linalg.norm(n_pt))

    compute_average_distance(distances)

max_dist = 100
min_dist = 3
variation = 1

white = (1.0, 1.0, 1.0, 0.5)
red = (1.0, 0.0, 0.0, 0.5)
blue = (0.0, 1.0, 0.0, 0.5)
green = (0.0, 0.0, 1.0, 0.5)

# Placement of the three receivers in 3D space
x_1 = -1.5/12
y_1 = 0.0
z_1 = 0.0
x_2 = (1.5/12)*np.cos(120)
y_2 = 0.0
z_2 = (1.5/12)*np.sin(120)
x_3 = (1.5/12)*np.cos(120)
y_3 = 0.0
z_3 = -(1.5/12)*np.sin(120)

# Generate receivers at x,y,z
rx1 = Receiver(x_1, y_1, z_1)
rx2 = Receiver(x_2, y_2, z_2)
rx3 = Receiver(x_3, y_3, z_3)

# Genrate 3D view
app = pg.mkQApp("GLScatterPlotItem Example")
w = gl.GLViewWidget()
w.showMaximized()
w.setWindowTitle('pyqtgraph example: GLScatterPlotItem')
w.setCameraPosition(distance=10, elevation=8, azimuth=270)

# Configure grid
g = gl.GLGridItem()
g.setSize(x=max_dist, y=max_dist, z=max_dist)
w.addItem(g)

# Plot receiver vectors on start
get_all_vectors()

# Set timer and callback
t = QtCore.QTimer()
t.timeout.connect(run_algorithm)
t.start(100)

if __name__ == '__main__':
    pg.exec()
