#!/usr/bin/env python3

import numpy as np

class Receiver:

    def __init__(self, x, y, z):

        self.x = x
        self.y = y
        self.z = z
        self.compute_vectors(90.0, 90.0)

    def compute_vectors(self, inc, azm):

        self.p = np.array([self.x, self.y, self.z])
        x = np.cos(azm)*np.sin(inc)
        y = np.sin(azm)*np.sin(inc)
        z = np.cos(inc)
        self.v = np.array([x,y,z])

        return [self.p, self.v]