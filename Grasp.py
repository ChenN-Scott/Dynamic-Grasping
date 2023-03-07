import pybullet as p
import pybullet_data
import time
import math
import os
import argparse
import numpy as np
<<<<<<< HEAD
from SIMENV import SimEnv
from Datasets import Datasets
=======
from simenv import SimEnv
from camera import Camera
from datasets import Datasets
>>>>>>> 3870a140ab17b02430b6bbaf10cfb236dd9700f7

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
<<<<<<< HEAD
    
    scene_path = 'scenes' + '\scene_%04d'%args.scene_id
    if not os.path.exists(scene_path):
        os.mkdir(scene_path)
    env = SimEnv(p, scene_path) # 初始化虚拟环境
=======

    # 初始化虚拟环境
    scene_path = 'scenes' + '\scene_%04d'%args.scene_id
    if not os.path.exists(scene_path):
        os.mkdir(scene_path)
    env = SimEnv(p, scene_path) 

    timeStep=1./1000.
    p.setTimeStep(timeStep)
>>>>>>> 3870a140ab17b02430b6bbaf10cfb236dd9700f7
    GRASP_STATE = False
    grasp_config = {'x':0, 'y':0, 'z':0.05, 'angle':0, 'width':0.08}
    # x y z width的单位是m, angle的单位是弧度

<<<<<<< HEAD
    while True:
        # 加载物体
        env.loadObjInURDF(args.object_name)
        if not os.path.exists(scene_path):
            os.mkdir(scene_path)
        
        # 保存相机内参和外参
        env.reset(mode=args.mode_name)
        env.save_Intrinsics(scene_path)
        env.save_Extrinsics(scene_path)

        kinect_path = os.path.join(scene_path,'kinect')

        time.sleep(0.5)
        start_time = time.time()
        last_time = time.time()
        while True:
            
            this_time = time.time()
            if this_time - last_time >= duration:
                p.stepSimulation()
                last_time = this_time
                env.renderURDFImage(save_path=kinect_path)
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
        break
=======
    # 加载物体
    target_id = env.loadObjInURDF(args.object_name)
    if not os.path.exists(scene_path):
        os.mkdir(scene_path)
    pos = env.reset(mode=args.mode_name)


    # 加载相机
    kinect_path = os.path.join(scene_path,'kinect')
    pose_point = pos[0]
    target_point = pos[1]
    head_point = pos[2]
    camera = Camera(p, kinect_path, pose_point, target_point, head_point)

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
        pass
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
>>>>>>> 3870a140ab17b02430b6bbaf10cfb236dd9700f7

if __name__ == "__main__":
    run()