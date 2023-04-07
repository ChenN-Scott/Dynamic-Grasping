import pybullet as p
import pybullet_data
import time
import math
import os
import argparse
import numpy as np
from simenv import SimEnv
from camera import Camera
from datasets import Datasets
import threading
from make_utils import obj_dict, find_obj_name

duration = 0.0667
GRASP_GAP = 0.005
GRASP_DEPTH = 0.005

def get_args():
    parser = argparse.ArgumentParser(description='Run Dynamic Grasping Experiment')
    parser.add_argument('--mode_name', type = str, default = 'dynamic_linear')
    parser.add_argument('--obj_name', type = str, default = 'mug')
    parser.add_argument('--scene_id', type = int, default = 0)
    args = parser.parse_args()
    return args

class MyThread(threading.Thread):
    def __init__(self, env):
        super(MyThread, self).__init__()  # 重构run函数必须要写
        self.env = env

    def run(self):
        start_time = time.time()
        while True:
            last_time = time.time()
            time.sleep(1./48000.)
            self.env.conveyor.step()
            this_time = time.time()
            if this_time - start_time > 25:
                break

def run():
    args = get_args()
    cid = p.connect(p.GUI)  # 连接服务器

    # 初始化虚拟环境
    scene_path = os.path.abspath('D:\data\scenes' + '\scene_%04d'%args.scene_id)
    if not os.path.exists(scene_path):
        os.mkdir(scene_path)
    env = SimEnv(p, scene_path) 

    timeStep=1./240.
    p.setTimeStep(timeStep)
    GRASP_STATE = False
    grasp_config = {'x':0, 'y':0, 'z':0.05, 'angle':0, 'width':0.08}
    # x y z width的单位是m, angle的单位是弧度

    # 加载物体
    object_name = find_obj_name(obj_dict, args.scene_id)
    target_id = env.loadObjInURDF(object_name)
    cam_pos = env.reset(mode=args.mode_name)
    py = env.get_p()

    # 加载相机
    kinect_path = os.path.join(scene_path,'kinect')
    pose_point = cam_pos[0]
    target_point = cam_pos[1]
    head_point = cam_pos[2]
    camera = Camera(py, kinect_path, pose_point, target_point, head_point)

    # 保存相机内参和外参
    camera.save_Intrinsics()
    camera.save_Extrinsics()

    # 开始仿真
    time.sleep(0.5)
    start_time = time.time()
    last_time = time.time()
    p.setRealTimeSimulation(1)
    thread = MyThread(env)
    thread.start()

    # 开始拍照
    start_time = time.time()
    last_time = time.time()
    while True:
        this_time = time.time()
        if this_time - last_time >= duration:
            print('{:8f}'.format(this_time - last_time)) 
            last_time = this_time
            camera.render_Image(py, target_id, env.conveyor.id)
        if this_time - start_time >= 25:
            break
    camera.save_data(object_name)

if __name__ == "__main__":
    run()
    #python Grasp.py -- 