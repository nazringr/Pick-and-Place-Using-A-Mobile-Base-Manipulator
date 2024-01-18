#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_srvs.srv import SetBool

import numpy as np

# IK Functions
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
class PickPlaceController(Node):
    def __init__(self):
        super().__init__('pick_place_controller')

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
            )

        self.odom_sub = self.create_subscription(PoseStamped, '/odom', self.odom_callback, self.qos_profile)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, self.qos_profile)
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/ur_controller/commands', 10)
        self.wheel_vel_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.steer_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.cli = self.create_client(SetBool, '/switch')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.t_delta = 20.0/1000.0 # seconds
        self.i = 0
        self.block_dist = .13
        self.x = 0
        self.y = 0

        # Initialize positions and set robot initial pose
        # [-0.3,0.7,-1.8,-0.75,-1.5707,0.0]
        self.Q = np.array([[-0.3],[0.7],[-1.8],[-0.85],[-1.5707],[0.0]])
        self.basePos = np.array([[0.0], [0.0], [0.0], [1.0]])
        ur_positions = Float64MultiArray()
        ur_positions.data = [self.Q[0,0], self.Q[1,0], self.Q[2,0], self.Q[3,0], self.Q[4,0], self.Q[5,0]]
        self.joint_position_pub.publish(ur_positions)

    def odom_callback(self, msg: PoseStamped):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y

    def lidar_callback(self, msg: LaserScan):
        ranges = msg.ranges

        if (min(ranges) > self.block_dist and min(ranges) < 2.0):
            wheel_vel_move = Float64MultiArray()
            wheel_vel_move.data = [1.0, 1.0]
            self.wheel_vel_pub.publish(wheel_vel_move)
        else:
            wheel_vel_stop = Float64MultiArray()
            wheel_vel_stop.data = [0.0, 0.0]
            self.wheel_vel_pub.publish(wheel_vel_stop)

        if (min(ranges) < self.block_dist):
            self.pick_place_action()

    def pick_place_action(self):
        self.destroy_subscription(self.lidar_sub)
        self.timer = self.create_timer(self.t_delta, self.drop_tool_pick)

    def raise_tool_pick(self):
        if self.i > 20:
            self.i = 0
            self.destroy_timer(self.timer)
            self.timer = self.create_timer(self.t_delta, self.drop_obj_home)
        else:
            z_dot = 0.05

            X_dot = np.array([[z_dot], [0.0], [0.0], [0.0], [0.0], [0.0]])
            # Calculate transformations and Jacobian using python functions
            # These are calculated using the current joint angles
            T_0to1 = createNumPyTransformMatrix(0.0, .15185, 0.0, np.pi/2)
            T_0to2 = np.matmul(T_0to1, createNumPyTransformMatrix(self.Q[1, 0]-np.pi/2, 0.0, -0.24355, 0))
            T_0to3 = np.matmul(T_0to2, createNumPyTransformMatrix(self.Q[2, 0], 0.0, -0.2132, 0))
            T_0to4 = np.matmul(T_0to3, createNumPyTransformMatrix(self.Q[3, 0]-np.pi/2, .13105, 0.0, np.pi/2))
            T_0to5 = np.matmul(T_0to4, createNumPyTransformMatrix(self.Q[4, 0]+np.pi/2, .08535, 0.0, -np.pi/2))
            T_0to6 = np.matmul(T_0to5, createNumPyTransformMatrix(self.Q[5, 0], 0.0, .0921+.1, 0.0))
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

            ur_positions = Float64MultiArray()
            ur_positions.data = [self.Q[0,0], self.Q[1,0], self.Q[2,0], self.Q[3,0], self.Q[4,0], self.Q[5,0]]
            self.joint_position_pub.publish(ur_positions)
            self.i += 1
    
    def drop_tool_pick(self):
        if self.i > 20:
            req = SetBool.Request()
            req.data = True
            self.cli.call_async(req)

            self.i = 0
            self.destroy_timer(self.timer)
            self.timer = self.create_timer(self.t_delta, self.raise_tool_pick)
        else:
            z_dot = -0.05

            X_dot = np.array([[z_dot], [0.0], [0.0], [0.0], [0.0], [0.0]])
            # Calculate transformations and Jacobian using python functions
            # These are calculated using the current joint angles
            T_0to1 = createNumPyTransformMatrix(0.0, .15185, 0.0, np.pi/2)
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

            ur_positions = Float64MultiArray()
            ur_positions.data = [self.Q[0,0], self.Q[1,0], self.Q[2,0], self.Q[3,0], self.Q[4,0], self.Q[5,0]]
            self.joint_position_pub.publish(ur_positions)
            self.i += 1

    def drop_obj_home(self):
        if self.y > 0.05:
            wheel_vel_move = Float64MultiArray()
            wheel_vel_move.data = [-1.0, -1.0]
            self.wheel_vel_pub.publish(wheel_vel_move)
        else:
            wheel_vel_stop = Float64MultiArray()
            wheel_vel_stop.data = [0.0, 0.0]
            self.wheel_vel_pub.publish(wheel_vel_stop)

            if self.i > 100:
                self.i = 0
                self.destroy_timer(self.timer)
                self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, self.qos_profile)
            elif self.i > 50:
                ur_positions = Float64MultiArray()
                ur_positions.data = [((self.i-50)*3.14/50)-3.14-0.314, self.Q[1,0], self.Q[2,0], self.Q[3,0], self.Q[4,0], self.Q[5,0]]
                self.joint_position_pub.publish(ur_positions)
                self.i += 1
            else:
                ur_positions = Float64MultiArray()
                ur_positions.data = [-self.i*3.14/50, self.Q[1,0], self.Q[2,0], self.Q[3,0], self.Q[4,0], self.Q[5,0]]
                self.joint_position_pub.publish(ur_positions)
                if self.i == 50:
                    req = SetBool.Request()
                    req.data = False
                    self.cli.call_async(req)
                self.i += 1
                

    
def main(args=None):
    rclpy.init(args=args)
    controller = PickPlaceController()
    
    try:
        rclpy.spin(controller)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
