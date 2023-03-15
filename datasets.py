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
    def __init__(self, kinect_path):
        self.kinect_path = kinect_path
        if not os.path.exists(self.kinect_path):
            os.mkdir(self.kinect_path)

        self.rgb_path = os.path.join(self.kinect_path,'rgb')
        if not os.path.exists(self.rgb_path):
            os.mkdir(self.rgb_path)

        self.depth_path = os.path.join(self.kinect_path,'depth')
        if not os.path.exists(self.depth_path):
            os.mkdir(self.depth_path)

        self.annotation_path = os.path.join(self.kinect_path,'annotation')
        if not os.path.exists(self.annotation_path):
            os.mkdir(self.annotation_path)

    def create_obj_id_list(self, object_name):
        np.savetxt('./obj_id_list', obj_dict[object_name], fmt="%d", delimiter="\n")

    def save_rgb(self, id, im_rgb):
        cv2.imwrite(self.rgb_path + '/%04d'%id+'.png', im_rgb)

    def save_depth(self, id, im_depth):
        cv2.imwrite(self.depth_path + '/%04d'%id+'.png', im_depth)

    def save_annotation(self, id, object_name, pos, ori):
        root = ET.Element('scene')
        tree = ET.ElementTree(root)
        obj = ET.Element("obj")
        obj_id = ET.Element("obj_id")
        obj_name = ET.Element("obj_name")
        obj_path = ET.Element("obj_path")
        pos_in_world = ET.Element("pos_in_world")
        ori_in_world = ET.Element("ori_in_world")
        obj_id.text = '{}'.format(obj_dict[object_name])
        obj_name.text = '{}.ply'.format(object_name)
        obj_path.text = 'Models/{}.ply'.format(object_name)
        pos_in_world.text = '{} {} {}'.format(pos[0],pos[1],pos[2])
        ori_in_world.text = '{} {} {} {}'.format(ori[0], ori[1], ori[2], ori[3])
        obj.append(obj_id)
        obj.append(obj_name)
        obj.append(obj_path)
        obj.append(pos_in_world)
        obj.append(ori_in_world)
        root.append(obj)
        self.indent(root)
        tree.write(self.annotation_path + '/%04d'%id + '.xml', encoding='utf-8', xml_declaration=True)

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

if __name__ == "__main__":
    data = Datasets('scenes\scene_0000')
    data.save_annotation(4,'mug',[6,7,8],[0,0,0,1])
