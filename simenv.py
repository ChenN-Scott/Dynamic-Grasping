import pybullet as p
import pybullet_data
import time
import math
import os
import random
import cv2
import numpy as np
import pybullet_utils as pu
import trimesh
from conveyor import Conveyor
from datasets import Datasets
import panda_sim_grasp as panda_sim
from math import pi, cos, sin, sqrt, atan, radians, degrees
import tool

class SimEnv():
    def __init__(self, bullet_client, scene_path):
        self.p = bullet_client
        self.p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        self.p.setPhysicsEngineParameter(maxNumCmdPer1ms=1000)
        self.p.resetDebugVisualizerCamera(cameraDistance=1.3, cameraYaw=38, cameraPitch=-22, cameraTargetPosition=[0, 0, 0])
        self.p.setAdditionalSearchPath(pybullet_data.getDataPath())  # 添加路径
        self.p.loadURDF("plane.urdf", [0, 0, 0])  # 加载地面
        self.p.setGravity(0, 0, -9.8) # 设置重力
        self.flags = self.p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
        self.p.setPhysicsEngineParameter(solverResidualThreshold=0)

        self.target_initial_pose = [[0.3, 0.3, 0], [0, 0, 0, 1]]
        self.robot_initial_pose = [[0, 0, 0], [0, 0, 0, 1]]
        self.target_id = 0

        # 初始化panda机器人
        # self.panda = panda_sim.PandaSimAuto(p, [0, 0, 0])
        self.scene_path = scene_path


        self.img_list = []

        # 加载传送带
        self.mesh_dir = os.path.abspath('Models')
        self.conveyor_speed = 0.5
        self.conveyor_urdf = os.path.abspath('Models/conveyor.urdf')
        self.conveyor_thickness = 0.02
        self.conveyor_initial_pose = [[0.3, 0.3, self.conveyor_thickness/2], [0, 0, 0, 1]]
        self.conveyor = Conveyor(self.conveyor_initial_pose, self.conveyor_urdf) 

    def get_p(self):
        return self.p

    def loadObjInURDF(self, object_name):
        """
        加载单个obj物体
        """
        object_mesh_filepath = os.path.join(self.mesh_dir, '{}'.format(object_name), '{}.obj'.format(object_name))   #物体的obj文件路径    
        target_mesh = trimesh.load_mesh(object_mesh_filepath)    #导入物体obj信息
        target_extents = target_mesh.bounding_box.extents.tolist()   #不知道干什么的
        floor_offset = target_mesh.bounds.min(0)[2]   #不知道干什么的        
        target_z = -target_mesh.bounds.min(0)[2] + self.conveyor_thickness   #物体的z坐标
        target_initial_pose = [[0.3, 0.3, target_z], [0, 0, 0, 1]]   #初始化物体位姿坐标
        target_urdf = 'Models/{}_target.urdf'.format(object_name)   #导入物体的urdf文件
        self.target_id = p.loadURDF(target_urdf, target_initial_pose[0], target_initial_pose[1])   #加载物体，获得物体id     
        p.setPhysicsEngineParameter(numSolverIterations=150, enableConeFriction=1, contactBreakingThreshold=1e-3)   
        return self.target_id

    def reset(self, mode, reset_dict=None):
        """
        初始化传送带的运动模式
        """
        if mode == 'initial':
            pu.remove_all_markers()
            target_pose = self.target_initial_pose
            conveyor_pose = [[target_pose[0][0], target_pose[0][1], self.conveyor_initial_pose[0][2]],
                             [0, 0, 0, 1]] if target_pose is not None else self.conveyor_initial_pose
            self.conveyor.set_pose(conveyor_pose)
            # self.panda.reset_robot()
            pu.step(2)
            return target_pose

        elif mode in ['dynamic_linear', 'dynamic_linear_vary_speed', 'dynamic_sinusoid']:
            pu.remove_all_markers()
            self.conveyor.clear_motion()  #清除传送带其他动作

            if reset_dict is None:  #对传送带参数重新采样
                distance, theta, length, direction = self.sample_convey_linear_motion()  #采样传送带线性运动参数
                target_quaternion = self.sample_target_angle()  #采样目标物体参数
                z_start_end = np.random.uniform(self.conveyor.conveyor_z_low, self.conveyor.conveyor_z_high, 2)
            else:
                distance, theta, length, direction, z_start_end = reset_dict['distance'], reset_dict['theta'], \
                                                                  reset_dict['length'], reset_dict['direction'], \
                                                                  reset_dict['z_start_end']
                target_quaternion = reset_dict['target_quaternion']

            self.conveyor.initialize_linear_motion(distance, theta, length, direction, self.conveyor_speed,      #初始化传送带线性运动
                                                    z_start_end[0], z_start_end[1],
                                                    variable_speed=mode == 'dynamic_linear_vary_speed')
            conveyor_pose = self.conveyor.start_pose
            target_z = self.target_initial_pose[0][2] - self.conveyor_initial_pose[0][2] + conveyor_pose[0][2]
            target_pose = [[conveyor_pose[0][0], conveyor_pose[0][1], target_z+0.1],
                           target_quaternion]
            self.conveyor.set_pose(conveyor_pose)
            time.sleep(0.1)
            p.resetBasePositionAndOrientation(self.target_id, target_pose[0], target_pose[1])
            # self.panda.reset_robot()
            pu.step(2)
            pu.draw_line(self.conveyor.start_pose[0], self.conveyor.target_pose[0])
            # p.resetDebugVisualizerCamera(cameraDistance=1.3, cameraYaw=theta + 90, cameraPitch=-35,
            #                              cameraTargetPosition=(0.0, 0.0, 0.0))
            # return ([self.conveyor.start_pose[0][0], self.conveyor.start_pose[0][1], 0.3], [self.conveyor.start_pose[0][0], self.conveyor.start_pose[0][1], 0], [0, 1, 0])
            return ([self.conveyor.target_pose[0][0], self.conveyor.target_pose[0][1], 0.5], 
                    [(self.conveyor.target_pose[0][0] + self.conveyor.start_pose[0][0])/2, (self.conveyor.target_pose[0][1] + self.conveyor.start_pose[0][1])/2, 0],
                    [(self.conveyor.target_pose[0][0] + self.conveyor.start_pose[0][0])/2-self.conveyor.start_pose[0][0], (self.conveyor.target_pose[0][1] + self.conveyor.start_pose[0][1])/2-self.conveyor.start_pose[0][1], 1]
            )
            # self.viewMatrix = self.p.computeViewMatrix([(self.conveyor.start_pose[0][0]+self.conveyor.target_pose[0][0])*1/3, 
            #                                             (self.conveyor.start_pose[0][1]+self.conveyor.target_pose[0][1])*1/3, 0.8], 
            #                                             [(self.conveyor.start_pose[0][0]+self.conveyor.target_pose[0][0])*1/3,
            #                                              (self.conveyor.start_pose[0][1]+self.conveyor.target_pose[0][1])*1/3, 0],
            #                                              [0, -1, 0])
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
            length = 1
        if direction is None:
            direction = random.sample([-1, 1], 1)[0]
        return dist, theta, length, direction

    @staticmethod
    def sample_target_angle():
        """ return quaternion """
        angle = np.random.uniform(-pi, pi)
        orientation = p.getQuaternionFromEuler([0, 0, angle])
        return list(orientation)

    def renderURDFImage(self, id, object_name):
        """
        渲染图像
        """
        target_pose, target_orn = p.getBasePositionAndOrientation(self.target_id)
        # self.datasets.save_annotation(id, object_name, target_pose, target_orn)

        # ======================== 渲染相机深度图 ========================
        # print('>> 渲染相机深度图...')
        # 渲染图像
        #img_camera = self.p.getCameraImage(IMAGEWIDTH, IMAGEHEIGHT, self.viewMatrix, self.projectionMatrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)
        # w = img_camera[0]      # width of the image, in pixels
        # h = img_camera[1]      # height of the image, in pixels
        # rgba = img_camera[2]    # color data RGB
        # dep = img_camera[3]    # depth data
        # # mask = img_camera[4]    # mask data

        # # 获取彩色图像
        # im_rgb = np.reshape(rgba, (h, w, 4))[:, :, [2, 1, 0]]
        # im_rgb = im_rgb.astype(np.uint8)

        # # 获取深度图像
        # depth = np.reshape(dep, (h, w))  # [40:440, 120:520]
        # A = np.ones((IMAGEHEIGHT, IMAGEWIDTH), dtype=np.float64) * farPlane * nearPlane
        # B = np.ones((IMAGEHEIGHT, IMAGEWIDTH), dtype=np.float64) * farPlane
        # C = np.ones((IMAGEHEIGHT, IMAGEWIDTH), dtype=np.float64) * (farPlane - nearPlane)
        # im_depthCamera = np.divide(A, (np.subtract(B, np.multiply(C, depth))))  # 单位 m

        # # 保存图像
        # # print('>> 保存相机深度图')
        # self.datasets.save_rgb(id, im_rgb)
        # self.datasets.save_depth(id, im_depthCamera)
        # print('>> 渲染结束')

    def img2camera(self, pt, dep):
        """
        获取像素点pt在相机坐标系中的坐标
        pt: [x, y]
        dep: 深度值

        return: [x, y, z]
        """
        pt_in_img = np.array([[pt[0]], [pt[1]], [1]], dtype=np.float)
        ret = np.matmul(np.linalg.inv(self.InMatrix), pt_in_img) * dep
        return list(ret.reshape((3,)))

    def camera2img(self, coord):
        """
        将相机坐标系中的点转换至图像
        coord: [x, y, z]
        return: [row, col]
        """
        z = coord[2]
        coord = np.array(coord).reshape((3, 1))
        rc = (np.matmul(self.InMatrix, coord) / z).reshape((3,))
        return list(rc)[:-1]
    
    def camera2world(self, coord):
        """
        获取相机坐标系中的点在世界坐标系中的坐标
        coord: [x, y, z]
        return: [x, y, z]
        """
        coord.append(1.)
        coord = np.array(coord).reshape((4, 1))
        coord_new = np.matmul(self.ExMatrix, coord).reshape((4,))
        return list(coord_new)[:-1]
    
    def world2camera(self, coord):
        """
        获取世界坐标系中的点在相机坐标系中的坐标
        coord: [x, y, z]
        return: [x, y, z]
        """
        coord.append(1.)
        coord = np.array(coord).reshape((4, 1))
        coord_new = np.matmul(np.linalg.inv(self.ExMatrix), coord).reshape((4,))
        return list(coord_new)[:-1]
    
    def world2img(self, coord):
        """
        获取世界坐标系中的点在图像中的坐标
        coord: [x, y, z]
        return: [row, col]
        """
        # 转到相机坐标系
        coord = self.world2camera(coord)
        # 转到图像
        pt = self.camera2img(coord) # [y, x]
        return [int(pt[1]), int(pt[0])]

    def img2world(self, pt, dep):
        """
        获取像素点的世界坐标
        pt: [x, y]
        dep: 深度值 m
        """
        coordInCamera = self.img2camera(pt, dep)
        return self.camera2world(coordInCamera)

if __name__ == "__main__":
    cid = p.connect(p.GUI)
    a = np.load('cam0_wrt_table.npy')
    b = np.load('camK.npy')
    c = np.load('camera_poses.npy')
    print(a.shape)
    print(b.shape)
    print(c.shape)
    env = SimEnv(p,'scenes\scene_0000')
    print(env.world2camera([0,1,2]))