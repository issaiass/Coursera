#!/usr/bin/python

import numpy as np

e = 0.17

class DiffDriveController():
    """
    Class used for controlling the robot linear and angular velocity
    """

    def __init__(self, max_speed, max_omega):
        # TODO for Student: Specify these parameters
        self.kp = 0.1
        self.ka = 0.4
        self.kb = -0.4
        self.MAX_SPEED = max_speed
        self.MAX_OMEGA = max_omega

    def compute_vel(self, state, goal):
        """
        Function that computes the desired outputs given the state and goal
        Inputs:
        state - a numpy vector of size 3 by 1 with components (x,y,theta)
        goal - a numpy vector of size 2 by 1 specifying the location of the goal
        Outputs: a tuple with 3 elements
        v - a number specifying the forward speed (in m/s) of the robot (should
            be no more than max_speed)
        omega - a number specifying the angular velocity (in rad/s) of the robot
            (should be no more than max_omega)
        done - a boolean value specifying if the robot has reached its goal (or
            is close enough
        """
        # YOUR CODE HERE
        done = False
        # state variables
        x = state[0]
        y = state[1]
        theta = state[2]
        # goal variables
        gx = goal[0]
        gy = goal[1]

        # displacement
        dx = x - gx
        dy = y - gy


        rho = (dx**2 + dy**2)**0.5
        alpha = -theta + np.math.atan2(dy, dx)
        beta = -theta - alpha

        A = np.array([[self.kp, 0, 0],[0, self.ka, self.kb]])
        B = np.array([[rho],[alpha],[beta]])
        vw = np.dot(A,B)
        v = vw.item(0)
        w = vw.item(1)
        if (v > self.MAX_SPEED):
            v = self.MAX_SPEED
        if (w > self.MAX_OMEGA):
            w = self.MAX_OMEGA
        if (abs(rho) <= e):
            done = True
            vw = [[0],[0]]
        print("v = {0}, w = {1}, rho = {2}, done = {3}".format(v, w, rho, done))
        return (v, w, done)
