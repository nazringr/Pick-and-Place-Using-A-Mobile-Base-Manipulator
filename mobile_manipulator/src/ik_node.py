#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import numpy as np
import matplotlib.pyplot as plt

x_plt = []
y_plt = []
z_plt = []

# New functions for efficiency, sympy is very slow so made all matrices initialized in numpy
def createNumPyTransformMatrix(theta, d, a, alpha):
    M = np.array([[np.cos(theta), (-np.sin(theta))*(np.cos(alpha)), (np.sin(theta))*(np.sin(alpha)), a*(np.cos(theta))],
        [(np.sin(theta)), (np.cos(theta))*(np.cos(alpha)), -(np.cos(theta))*(np.sin(alpha)), a*(np.sin(theta))],
        [0.0, (np.sin(alpha)), (np.cos(alpha)), d],
        [0.0, 0.0, 0.0, 1.0]])
    return M

def createNumpyJacobianMatrix(listOfTransforms):
    j_comps = []
    z_prev = np.array([[0.0], [0.0], [1.0]])
    o_prev = np.array([[0.0], [0.0], [0.0]])
    o_n = np.array([[listOfTransforms[-1][0, 3]], [listOfTransforms[-1][1, 3]], [listOfTransforms[-1][2, 3]]])
    for transform in listOfTransforms:
        l = o_n - o_prev
        ZxL = np.cross(z_prev, l, axis=0)
        j = np.vstack((ZxL, z_prev))
        j_comps.append(j)
        z_prev = np.array([[transform[0, 2]], [transform[1, 2]], [transform[2,2]]])
        o_prev = np.array([[transform[0, 3]], [transform[1, 3]], [transform[2,3]]])
    return np.hstack((j_comps[0], j_comps[1], j_comps[2], j_comps[3], j_comps[4], j_comps[5]))

# Controller node
class IKController(Node):
    def __init__(self):
        super().__init__('ik_controller')

        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/ur_controller/commands', 10)
        self.t_delta = 20.0/1000.0 # seconds
        self.timer = self.create_timer(self.t_delta, self.timer_callback)
        self.i = 0

        # Start plotting and calculation loop
        self.Q = np.array([[0.0],[0.5],[-1.5],[-0.45],[-1.5707],[0.0]])
        self.basePos = np.array([[0.0], [0.0], [0.0], [1.0]])

    def timer_callback(self):
        if self.i > 200:
            raise SystemExit
        
        z_dot = -0.075
        if self.i > 100:
            z_dot = 0.075

        X_dot = np.array([[z_dot], [0.0], [0.0], [0.0], [0.0], [0.0]])
        # Calculate transformations and Jacobian using python functions
        # These are calculated using the current joint angles
        T_0to1 = createNumPyTransformMatrix(self.Q[0, 0], .15185, 0.0, np.pi/2)
        T_0to2 = np.matmul(T_0to1, createNumPyTransformMatrix(self.Q[1, 0]-np.pi/2, 0.0, -0.24355, 0))
        T_0to3 = np.matmul(T_0to2, createNumPyTransformMatrix(self.Q[2, 0], 0.0, -0.2132, 0))
        T_0to4 = np.matmul(T_0to3, createNumPyTransformMatrix(self.Q[3, 0]-np.pi/2, .13105, 0.0, np.pi/2))
        T_0to5 = np.matmul(T_0to4, createNumPyTransformMatrix(self.Q[4, 0]+np.pi/2, .08535, 0.0, -np.pi/2))
        T_0to6 = np.matmul(T_0to5, createNumPyTransformMatrix(self.Q[5, 0], 0.0, .0921, 0.0))
        list_of_transforms = [T_0to1, T_0to2, T_0to3, T_0to4, T_0to5, T_0to6]
        # Calculate current jacobian matrix using transformation matrices and first method in lecture
        currJacobian = createNumpyJacobianMatrix(list_of_transforms)
        # invert jacobian matrix to get Q from X
        invJacobian = np.linalg.pinv(currJacobian)
        # Use inverse jacobian and X_dot to get joint velocities
        # Then multiply by time delta and add (aka Numerical Integration)
        Q_dot = np.matmul(invJacobian, X_dot)
        Q_delta = self.t_delta*Q_dot
        self.Q = self.Q + Q_delta
        
        endPosition = np.matmul(T_0to6, self.basePos)
        x_plt.append(endPosition[0])
        y_plt.append(endPosition[1])
        z_plt.append(endPosition[2])

        ur_positions = Float64MultiArray()
        ur_positions.data = [self.Q[0,0], self.Q[1,0], self.Q[2,0], self.Q[3,0], self.Q[4,0], self.Q[5,0]]
        self.joint_position_pub.publish(ur_positions)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    controller = IKController()
    
    try:
        rclpy.spin(controller)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    controller.destroy_node()
    rclpy.shutdown()

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.scatter3D(z_plt, y_plt, x_plt)
    plt.title("Robot End Trajectory")
    ax.set_xlabel('X-axis', fontweight ='bold')
    ax.set_ylabel('Y-axis', fontweight ='bold')
    ax.set_zlabel('Z-axis', fontweight ='bold')
    ax.set_xlim((-0.5, 0.5))
    ax.set_ylim((-0.5, 0.5))
    ax.set_zlim((-0.5, 0.5))
    plt.show()


if __name__ == '__main__':
    main()
