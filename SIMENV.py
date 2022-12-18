import pybullet as p
import pybullet_data
import time
import math
import os
import glob
import random
import cv2
import shutil
import numpy as np
import scipy.io as scio
import pybullet_utils as pu
import trimesh
import misc_utils as mu
from Conveyor import Conveyor
import panda_sim_grasp as panda_sim
import grasp_utils as gu
from math import pi, cos, sin, sqrt, atan, radians, degrees
import tool

# 图像尺寸
IMAGEWIDTH = 640
IMAGEHEIGHT = 480

nearPlane = 0.01
farPlane = 10

fov = 60    # 垂直视场 图像高tan(30) * 0.7 *2 = 0.8082903m
aspect = IMAGEWIDTH / IMAGEHEIGHT

size=(0.8, 0.8)     # 桌面深度图实际尺寸 m
unit=0.0002          # 每个像素的长度 0.1mm


class SimEnv():
    def __init__(self, bullet_client):
        self.p = bullet_client
        self.p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        self.p.setPhysicsEngineParameter(maxNumCmdPer1ms=1000)
        self.p.resetDebugVisualizerCamera(cameraDistance=1.3, cameraYaw=38, cameraPitch=-22, cameraTargetPosition=[0, 0, 0])
        self.p.setAdditionalSearchPath(pybullet_data.getDataPath())  # 添加路径
        self.p.loadURDF("plane.urdf", [0, 0, 0])  # 加载地面
        self.p.setGravity(0, 0, -9.8) # 设置重力
        self.flags = self.p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
        self.p.setPhysicsEngineParameter(solverResidualThreshold=0)

        global image_id
        image_id = 0
        # 初始化panda机器人
        self.panda = panda_sim.PandaSimAuto(p, [0, 0, 0])

        self.world_steps = 0
        self.target_initial_pose = [[0.3, 0.3, 0], [0, 0, 0, 1]]
        self.robot_initial_pose = [[0, 0, 0], [0, 0, 0, 1]]

        # 加载相机
        self.viewMatrix = self.p.computeViewMatrix([0, 0, 0.7], [0, 0, 0], [0, 1, 0])
        self.projectionMatrix = self.p.computeProjectionMatrixFOV(fov, aspect, nearPlane, farPlane)

        # 加载传送带
        self.mesh_dir = os.path.abspath('Models')
        self.conveyor_speed = 0.2
        self.conveyor_urdf = os.path.abspath('Models/conveyor.urdf')
        self.conveyor_thickness = 0.02
        self.conveyor_initial_pose = [[0.3, 0.3, self.conveyor_thickness/2], [0, 0, 0, 1]]
        self.conveyor = Conveyor(self.conveyor_initial_pose, self.conveyor_urdf)

        self.target = 0


    def loadObjInURDF(self, object_name):
        """
        以URDF的格式加载单个obj物体

        idx: 物体id
        """
        # 获取物体文件

        self.urdfs_filename = os.path.abspath('Models/{}'.format(object_name))
        print('urdf filename = ', self.urdfs_filename[0])
        
        object_mesh_filepath = os.path.join(self.mesh_dir, '{}'.format(object_name), '{}.obj'.format(object_name))
        
        target_mesh = trimesh.load_mesh(object_mesh_filepath)
        target_extents = target_mesh.bounding_box.extents.tolist()
        floor_offset = target_mesh.bounds.min(0)[2]
        
        target_z = -target_mesh.bounds.min(0)[2] + self.conveyor_thickness
        target_initial_pose = [[0.3, 0.3, target_z], [0, 0, 0, 1]]
        print('object_mesh_filepath: ',object_mesh_filepath)
        # target_urdf = mu.create_object_urdf(object_mesh_filepath, object_name,
        #                                 urdf_target_object_filepath='Models/{}_target.urdf'.format(object_name))
        target_urdf = 'Models/{}_target.urdf'.format(object_name)
        self.target = p.loadURDF(target_urdf, target_initial_pose[0], target_initial_pose[1])
        
        p.setPhysicsEngineParameter(numSolverIterations=150, enableConeFriction=1, contactBreakingThreshold=1e-3)

    def reset(self, mode, reset_dict=None):
        self.world_steps = 0
        self.value_markers = None
        conveyor_z_low = 0.01
        conveyor_z_high = 0.01
        if mode == 'initial':
            pu.remove_all_markers()
            # target_pose, distance = self.target_initial_pose, self.initial_distance
            target_pose = self.target_initial_pose
            conveyor_pose = [[target_pose[0][0], target_pose[0][1], self.conveyor_initial_pose[0][2]],
                             [0, 0, 0, 1]] if target_pose is not None else self.conveyor_initial_pose
            self.conveyor.set_pose(conveyor_pose)
            self.panda.reset_robot()
            pu.step(2)
            return target_pose

        elif mode in ['dynamic_linear', 'dynamic_linear_vary_speed', 'dynamic_sinusoid']:
            pu.remove_all_markers()

            self.conveyor.clear_motion()

            if reset_dict is None:
                distance, theta, length, direction = self.sample_convey_linear_motion()
                target_quaternion = self.sample_target_angle()
                z_start_end = np.random.uniform(conveyor_z_low, conveyor_z_high, 2)
            else:
                distance, theta, length, direction, z_start_end = reset_dict['distance'], reset_dict['theta'], \
                                                                  reset_dict['length'], reset_dict['direction'], \
                                                                  reset_dict['z_start_end']
                target_quaternion = reset_dict['target_quaternion']

            self.conveyor.initialize_linear_motion(distance, theta, length, direction, self.conveyor_speed,
                                                    z_start_end[0], z_start_end[1],
                                                    variable_speed=mode == 'dynamic_linear_vary_speed')
            conveyor_pose = self.conveyor.start_pose
            target_z = self.target_initial_pose[0][2] - self.conveyor_initial_pose[0][2] + conveyor_pose[0][2]
            target_pose = [[conveyor_pose[0][0], conveyor_pose[0][1], target_z+0.1],
                           target_quaternion]
            self.conveyor.set_pose(conveyor_pose)
            time.sleep(0.1)
            p.resetBasePositionAndOrientation(self.target, target_pose[0], target_pose[1])
            

            self.panda.reset_robot()
            pu.step(2)

            pu.draw_line(self.conveyor.start_pose[0], self.conveyor.target_pose[0])

            p.resetDebugVisualizerCamera(cameraDistance=1.3, cameraYaw=theta + 90, cameraPitch=-35,
                                         cameraTargetPosition=(0.0, 0.0, 0.0))
            return distance, theta, length, direction, target_quaternion, np.array(z_start_end).tolist()

        elif mode == 'hand_over':
            raise NotImplementedError
        else:
            raise NotImplementedError

    def sample_convey_linear_motion(self, dist=None, theta=None, length=None, direction=None):
        """ theta is in degrees """
        distance_low = 0.15
        distance_high = 0.25
        if dist is None:
            dist = np.random.uniform(low=distance_low, high=distance_high)
        if theta is None:
            theta = np.random.uniform(low=-90, high=90)
        if length is None:
            length = 1.0
        if direction is None:
            direction = random.sample([-1, 1], 1)[0]
        return dist, theta, length, direction

    @staticmethod
    def sample_target_angle():
        """ return quaternion """
        angle = np.random.uniform(-pi, pi)
        orientation = p.getQuaternionFromEuler([0, 0, angle])
        return list(orientation)

    def renderURDFImage(self, save_path):
        """
        渲染图像
        """
        global image_id
        if not os.path.exists(save_path):
            os.mkdir(save_path)
        save_path = os.path.join(save_path,'{}'.format(image_id))
        if not os.path.exists(save_path):
            os.mkdir(save_path)

        # ======================== 渲染相机深度图 ========================
        print('>> 渲染相机深度图...')
        # 渲染图像
        img_camera = self.p.getCameraImage(IMAGEWIDTH, IMAGEHEIGHT, self.viewMatrix, self.projectionMatrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)
        w = img_camera[0]      # width of the image, in pixels
        h = img_camera[1]      # height of the image, in pixels
        rgba = img_camera[2]    # color data RGB
        dep = img_camera[3]    # depth data
        mask = img_camera[4]    # mask data

        # 获取彩色图像
        im_rgb = np.reshape(rgba, (h, w, 4))[:, :, [2, 1, 0]]
        im_rgb = im_rgb.astype(np.uint8)

        # 获取深度图像
        depth = np.reshape(dep, (h, w))  # [40:440, 120:520]
        A = np.ones((IMAGEHEIGHT, IMAGEWIDTH), dtype=np.float64) * farPlane * nearPlane
        B = np.ones((IMAGEHEIGHT, IMAGEWIDTH), dtype=np.float64) * farPlane
        C = np.ones((IMAGEHEIGHT, IMAGEWIDTH), dtype=np.float64) * (farPlane - nearPlane)
        # im_depthCamera = A / (B - C * depth)  # 单位 m
        im_depthCamera = np.divide(A, (np.subtract(B, np.multiply(C, depth))))  # 单位 m
        im_depthCamera_rev = np.ones((IMAGEHEIGHT, IMAGEWIDTH), dtype=np.float) * im_depthCamera.max() - im_depthCamera # 反转深度

        # 获取分割图像
        im_mask = np.reshape(mask, (h, w))

        # 保存图像
        # print('>> 保存相机深度图')
        # scio.savemat(save_path + '/camera_rgb{}.mat'.format(image_id), {'A':im_rgb})
        # scio.savemat(save_path + '/camera_depth{}.mat'.format(image_id), {'A':im_depthCamera})
        # scio.savemat(save_path + '/camera_depth_rev{}.mat'.format(image_id), {'A':im_depthCamera_rev})
        # scio.savemat(save_path + '/camera_mask{}.mat'.format(image_id), {'A':im_mask})

        cv2.imwrite(save_path + '/camera_rgb{}.png'.format(image_id), im_rgb)
        # cv2.imwrite(save_path + '/camera_mask{}.png'.format(image_id), im_mask*20)
        cv2.imwrite(save_path + '/camera_depth{}.png'.format(image_id), tool.depth2Gray(im_depthCamera))
        # cv2.imwrite(save_path + '/camera_depth_rev{}.png'.format(image_id), tool.depth2Gray(im_depthCamera_rev))
        image_id = image_id+1
        print('>> 渲染结束')