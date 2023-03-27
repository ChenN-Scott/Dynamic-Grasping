import os
import numpy as np
import pybullet as p
import pybullet_data
import time
import pybullet_utils as pu
import threading
from math import pi, cos, sin, sqrt, atan, radians, degrees
import random
import trimesh
from itertools import combinations
from collections import deque
import torch
import torch.nn.functional as F

class Conveyor:
    def __init__(self, initial_pose, urdf_path):
        self.initial_pose = initial_pose
        self.urdf_path = urdf_path
        self.id = p.loadURDF(self.urdf_path, initial_pose[0], initial_pose[1])
        self.cid = p.createConstraint(parentBodyUniqueId=self.id, parentLinkIndex=-1, childBodyUniqueId=-1,
                                      childLinkIndex=-1, jointType=p.JOINT_FIXED, jointAxis=[0, 0, 0],
                                      parentFramePosition=[0, 0, 0], childFramePosition=initial_pose[0],
                                      childFrameOrientation=initial_pose[1])

        # motion related
        self.start_pose = None
        self.target_pose = None
        self.discretized_trajectory = None
        self.wp_target_index = 0
        self.distance = None
        self.theta = None
        self.length = None
        self.direction = None
        self.speed = None
        self.z_start = None
        self.z_end = None
        self.circular_angles = None
        self.conveyor_z_low = 0.01
        self.conveyor_z_high = 0.01

    def set_pose(self, pose):
        pu.set_pose(self.id, pose)
        self.control_pose(pose)

    def get_pose(self):
        return pu.get_body_pose(self.id)

    def control_pose(self, pose):
        p.changeConstraint(self.cid, jointChildPivot=pose[0], jointChildFrameOrientation=pose[1])

    def step(self):
        if self.discretized_trajectory is None or self.wp_target_index == len(self.discretized_trajectory):
            pass
        else:
            self.control_pose(self.discretized_trajectory[self.wp_target_index])
            self.wp_target_index += 1

    def initialize_linear_motion(self, dist, theta, length, direction, speed, z_start, z_end, variable_speed=False):
        """
        :param dist: distance to robot center,
        :param theta: the angle of rotation, (0, 360)
        :param length: the length of the motion
        :param direction: the direction of the motion
            1: from smaller theta to larger theta
            -1: from larger theta to smaller theta
        :param z_start: the height of the conveyor at the start
        :param z_end: the height of the conveyor at the end
        :param speed: the speed of the conveyor
        :param variable_speed: determines if the speed of the conveyor is variable or constant
        """
        self.distance = float(dist)
        self.theta = float(theta)
        self.length = float(length)
        self.direction = float(direction)
        self.speed = float(speed)
        self.z_start = float(z_start)
        self.z_end = float(z_end)
        # use the orientation of the initial pose
        orientation = self.initial_pose[1]
        # compute start xy and end xy
        new_dist = sqrt(dist ** 2 + (length / 2.0) ** 2)
        delta_theta = atan((length / 2.0) / dist)

        theta_large = radians(self.theta) + delta_theta
        theta_small = radians(self.theta) - delta_theta

        if direction == -1:
            start_xy = [new_dist * cos(theta_large), new_dist * sin(theta_large)]
            target_xy = [new_dist * cos(theta_small), new_dist * sin(theta_small)]
        elif direction == 1:
            target_xy = [new_dist * cos(theta_large), new_dist * sin(theta_large)]
            start_xy = [new_dist * cos(theta_small), new_dist * sin(theta_small)]
        else:
            raise ValueError('direction must be in {-1, 1}')
        start_position = start_xy + [self.z_start]
        target_position = target_xy + [self.z_end]

        self.start_pose = [start_position, orientation]
        self.target_pose = [target_position, orientation]

        if variable_speed:
            # acc_levels = 2     # accelerate / decelerate level e.g. acc_levels = 3 -> [1/3, 1/2, 1, 2, 3]
            # speed_multipliers = set(np.concatenate((np.arange(1, acc_levels+1), 1./np.arange(1, acc_levels+1))))
            # speeds = np.array(sorted(speed_multipliers)) * self.speed
            n_segments = 10  # num speed switches
            # speed_multipliers = np.array(list(range(1, n_segments//2 + 1)) * 2) / (n_segments/2.)
            # rng = np.random.RandomState(2)
            # speeds = rng.permutation(speed_multipliers) * self.speed
            speed_multipliers = np.linspace(0.6, 1.0, n_segments)[::-1]
            speeds = speed_multipliers * self.speed
            segments = np.linspace(start_position, target_position, n_segments+1)
            position_trajectory = []
            for i in range(n_segments):
                # speed = np.random.choice(speeds)
                speed = speeds[i]
                dist = np.linalg.norm(segments[i] - segments[i+1])
                num_steps = int(dist / speed * 240)
                wps = np.linspace(segments[i], segments[i+1], num_steps)
                position_trajectory.extend(wps)
        else:
            num_steps = int(self.length / self.speed * 480)
            position_trajectory = np.linspace(start_position, target_position, num_steps)
        self.discretized_trajectory = [[list(pos), orientation] for pos in position_trajectory]
        self.wp_target_index = 1

    def initialize_sinusoid_motion(self, dist, theta, length, direction, speed, amp_div=8, period_div=3):
        """
        :param dist: distance to robot center,
        :param theta: the angle of rotation, (0, 360)
        :param length: the length of the motion
        :param direction: the direction of the motion
            1: from smaller theta to larger theta
            -1: from larger theta to smaller theta
        :param speed: the speed of the conveyor
        """
        self.distance = float(dist)
        self.theta = float(theta)
        self.length = float(length)
        self.direction = float(direction)
        self.speed = float(speed)
        # uses the z value and orientation of the current pose
        z = self.get_pose()[0][-1]
        orientation = self.get_pose()[1]

        num_steps = int(self.length / self.speed * 240)

        start_position = np.array([0, -length / 2.0, 1]) * direction
        target_position = np.array([0, length / 2.0, 1]) * direction
        position_trajectory = np.linspace(start_position, target_position, num_steps)
        # Amplitude: length/4., period: self.length/3 i.e. 3 sinusoids within the length of the trajectory
        position_trajectory[:, 0] = (self.length * 1.0 / amp_div) * np.sin(2 * np.pi * position_trajectory[:, 1] /
                                                                           (self.length * 1.0 / period_div))
        T_1 = np.array([[np.cos(radians(self.theta)), -np.sin(radians(self.theta)), 0],
                        [np.sin(radians(self.theta)), np.cos(radians(self.theta)), 0],
                        [0, 0, 1]])
        T_2 = np.array([[1, 0, self.distance], [0, 1, 0], [0, 0, 1]])
        position_trajectory = np.dot(T_1, np.dot(T_2, position_trajectory.T)).T
        position_trajectory[:, -1] = z
        self.start_pose = [position_trajectory[0], orientation]
        self.target_pose = [position_trajectory[-1], orientation]

        self.discretized_trajectory = [[list(pos), orientation] for pos in position_trajectory]
        self.wp_target_index = 1

    def initialize_circular_motion(self, dist, theta, length, direction, speed):
        """
        :param dist: distance to robot center,
        :param theta: the angle of rotation, (0, 360)
        :param length: the length of the motion
        :param direction: the direction of the motion
            1: counter clockwise
            -1: clockwise
        :param speed: the speed of the conveyor
        """
        self.distance = float(dist)
        self.theta = float(theta)
        self.length = float(length)
        self.direction = float(direction)
        self.speed = float(speed)
        # uses the z value and orientation of the current pose
        z = self.get_pose()[0][-1]
        orientation = self.get_pose()[1]

        # calculate waypoints
        num_points = int(self.length / self.speed) * 240
        delta_angle = self.length / self.distance
        angles = np.linspace(radians(theta), radians(theta)+delta_angle, num_points)
        if direction == -1:
            angles = angles[::-1]
        self.circular_angles = angles

        self.discretized_trajectory = [[(cos(ang) * self.distance, sin(ang) * self.distance, z), orientation] for ang in angles]
        self.wp_target_index = 1

        self.start_pose = self.discretized_trajectory[0]
        self.target_pose = self.discretized_trajectory[-1]

    # def initialize_conveyor_motion_v2(self, angle, speed, length, start_pose=None, is_sinusoid=False):
    #     """
    #     Initialize a motion using the start pose as initial pose, in the direction of the angle.

    #     :param angle: the angle of the motion direction in the conveyor frame, in degrees
    #     :param speed: the speed of the motion
    #     """
    #     self.theta = float(angle)
    #     self.length = float(length)
    #     self.speed = float(speed)
    #     start_pose_in_world = conveyor_pose = self.get_pose() if start_pose is None else start_pose

    #     num_steps = int(length / speed * 240)

    #     start_position = np.array([0, 0, 1])
    #     target_position = np.array([length, 0, 1])
    #     position_trajectory = np.linspace(start_position, target_position, num_steps)

    #     if is_sinusoid:
    #         # Amplitude: dist/2., period: self.length/3 i.e. 3 sinusoids within the length of the trajectory
    #         position_trajectory[:, 1] = self.length/20. * np.sin(2*np.pi * position_trajectory[:, 0] / (self.length/4))

    #     # rotate direction of motion according to angle
    #     T_1 = np.array([[np.cos(radians(self.theta)), -np.sin(radians(self.theta)), 0],
    #                     [np.sin(radians(self.theta)), np.cos(radians(self.theta)), 0],
    #                     [0, 0, 1]])
    #     position_trajectory = np.dot(T_1, position_trajectory.T).T
    #     position_trajectory[:, -1] = 0

    #     # adjust motion to the reference start position
    #     position_trajectory = np.dot(tfc.toMatrix(tfc.fromTf(start_pose_in_world)),
    #                                  np.concatenate((position_trajectory, np.ones((position_trajectory.shape[0], 1))),
    #                                                 axis=-1).T)[:-1].T

    #     self.discretized_trajectory = [[list(p), start_pose_in_world[1]] for p in position_trajectory]
    #     self.wp_target_index = 1
    #     return self.discretized_trajectory[0], self.discretized_trajectory[-1]

    def clear_motion(self):
        self.start_pose = None
        self.target_pose = None
        self.discretized_trajectory = None
        self.wp_target_index = 0
        self.distance = None
        self.theta = None
        self.length = None
        self.direction = None
        self.circular_angles = None

    def predict(self, duration):
        # predict the ground truth future pose of the conveyor
        num_predicted_steps = int(duration * 960)
        predicted_step_index = self.wp_target_index - 1 + num_predicted_steps
        if predicted_step_index < len(self.discretized_trajectory):
            return self.discretized_trajectory[predicted_step_index]
        else:
            return self.discretized_trajectory[-1]


