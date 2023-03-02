import numpy as np
import os
import cv2
import tool
import xml.etree.ElementTree as ET
from xml.etree.ElementTree import Element

obj_dict = {'bleach_cleanser':1,
            'bowl':2,
            'cracker_box':3,
            'cube':4,
            'master_chef_can':5,
            'mug':6,
            'mustard_bottle':7,
            'potted_meat_can':8,
            'power_drill':9,
            'power_drill_bkup':10,
            'pudding_box':11,
            'sugar_box':12,
            'tomato_soup_can':13}

class Datasets():
    def __init__(self, scene_path):
        self.root = ET.Element('scene')
        self.tree = ET.ElementTree(self.root)
        self.obj = ET.Element("obj")
        self.obj_id = ET.Element("obj_id")
        self.obj_name = ET.Element("obj_name")
        self.obj_path = ET.Element("obj_path")
        self.pos_in_world = ET.Element("pos_in_world")
        self.ori_in_world = ET.Element("ori_in_world")

        self.kinect_path = os.path.join(scene_path,'\kinect')
        if not os.path.exists(self.kinect_path):
            os.mkdir(self.kinect_path)

        self.rgb_path = os.path.join(self.kinect_path,'\rgb')
        if not os.path.exists(self.rgb_path):
            os.mkdir(self.rgb_path)

        self.depth_path = os.path.join(self.kinect_path,'\depth')
        if not os.path.exists(self.depth_path):
            os.mkdir(self.depth_path)

        self.annotation_path = os.path.join(self.kinect_path,'\annotation')
        if not os.path.exists(self.depth_path):
            os.mkdir(self.depth_path)

    def create_obj_id_list(self, object_name):
        np.savetxt('./obj_id_list', obj_dict[object_name], fmt="%d", delimiter="\n")

    def save_rgb(self, im_rgb, id):
        cv2.imwrite(self.rgb_path + '/%04d{}.png'.format(id), im_rgb)

    def save_depth(self, im_depth, id):
        cv2.imwrite(self.depth_path + '/%04d{}.png'.format(id), tool.depth2Gray(im_depth))

    def save_annotation(self, id, object_name, pos, ori):
        self.obj_id.text = obj_dict[object_name]
        self.obj_name.text = '{}.ply'.format(object_name)
        self.obj_path.text = 'Models/{}.ply'.format(object_name)
        self.pos_in_world.text = '{} {} {}'.format(pos[0],pos[1],pos[2])
        self.ori_in_world.text = '{} {} {} {}'.format(ori[0], ori[1], ori[2], ori[3])
        self.obj.append(self.obj_id)
        self.obj.append(self.obj_name)
        self.obj.append(self.obj_path)
        self.obj.append(self.pos_in_world)
        self.obj.append(self.ori_in_world)
        self.root.append(self.obj)
        self.indent(self.root)
        self.tree.write("%04d{}.xml".format(id), encoding='utf-8', xml_declaration=True)

    def save_pos_ori(self, target_pose, target_orn):
        pass

    def indent(self, elem, level=0):
        i = "\n" + level*"\t"
        if len(elem):
            if not elem.text or not elem.text.strip():
                elem.text = i + "\t"
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
            for elem in elem:
                self.indent(elem, level+1)
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
        else:
            if level and (not elem.tail or not elem.tail.strip()):
                elem.tail = i
    
