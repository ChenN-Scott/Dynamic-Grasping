import pybullet as p
import pybullet_data
import time
import math
import os
import argparse
import numpy as np
from SIMENV import SimEnv
image_id = 0

GRASP_GAP = 0.005
GRASP_DEPTH = 0.005

def get_args():
    parser = argparse.ArgumentParser(description='Run Dynamic Grasping Experiment')
    parser.add_argument('--object_name', type=str, default='bleach_cleanser')           #要抓取的物体名称\
    parser.add_argument('--mode_name', type=str, default='dynamic_linear')
    args = parser.parse_args()
    return args

def run():
    args = get_args()
    cid = p.connect(p.GUI)  # 连接服务器
    env = SimEnv(p) # 初始化虚拟环境
    global image_id

    GRASP_STATE = False
    grasp_config = {'x':0, 'y':0, 'z':0.05, 'angle':0, 'width':0.08}
    # x y z width的单位是m, angle的单位是弧度
    img_path = 'Images\{}'.format(args.object_name)

    while True:
        # 加载物体
        env.loadObjInURDF(args.object_name)
        print(img_path)
        if not os.path.exists(img_path):
            os.mkdir(img_path)
        env.reset(mode=args.mode_name)
        while True:
            p.stepSimulation()
            time.sleep(1./2400.)
            env.conveyor.step()
            env.renderURDFImage(save_path=img_path)
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