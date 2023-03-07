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

duration = 0.0333
GRASP_GAP = 0.005
GRASP_DEPTH = 0.005

def get_args():
    parser = argparse.ArgumentParser(description='Run Dynamic Grasping Experiment')
    parser.add_argument('--object_name', type=str, default='bleach_cleanser')           #要抓取的物体名称\
    parser.add_argument('--mode_name', type=str, default='dynamic_linear')
    parser.add_argument('--scene_id', type=int, default=0)
    args = parser.parse_args()
    return args

def run():
    args = get_args()
    cid = p.connect(p.GUI)  # 连接服务器

    # 初始化虚拟环境
    scene_path = 'scenes' + '\scene_%04d'%args.scene_id
    if not os.path.exists(scene_path):
        os.mkdir(scene_path)
    env = SimEnv(p, scene_path) 

    # 加载相机
    kinect_path = os.path.join(scene_path,'kinect')
    pose_point = [0, 0, 0.15]
    target_point = [0, 0, 0]
    head_point = [0, 1, 1]
    camera = Camera(p, kinect_path, pose_point, target_point, head_point)

    timeStep=1./1000.
    p.setTimeStep(timeStep)
    GRASP_STATE = False
    grasp_config = {'x':0, 'y':0, 'z':0.05, 'angle':0, 'width':0.08}
    # x y z width的单位是m, angle的单位是弧度

    # 加载物体
    target_id = env.loadObjInURDF(args.object_name)
    if not os.path.exists(scene_path):
        os.mkdir(scene_path)
    env.reset(mode=args.mode_name)

    # 保存相机内参和外参
    camera.save_Intrinsics()
    camera.save_Extrinsics()

    # 开始仿真
    time.sleep(0.5)
    start_time = time.time()
    last_time = time.time()
    p.setRealTimeSimulation(1)
    while True:
        this_time = time.time()
        print('{:8f}'.format(this_time - last_time))
        if this_time - last_time >= duration:
            last_time = this_time
            camera.render_Image(target_id)
        time.sleep(1./24000.)
        if this_time - start_time >= 10:
            break
        env.conveyor.step()
        
        # 检测按键
        # keys = p.getKeyboardEvents()
        # if ord('1') in keys and keys[ord('1')]&p.KEY_WAS_TRIGGERED:
        #     # 渲染图像
        #     env.renderURDFImage(save_path=img_path)
        # if ord('2') in keys and keys[ord('2')]&p.KEY_WAS_TRIGGERED:
        #     env.conveyor.step()
        # # 按3重置环境            
        # if ord('3') in keys and keys[ord('3')]&p.KEY_WAS_TRIGGERED:
        #     env.removeObjsInURDF()
        #     break

if __name__ == "__main__":
    run()