import numpy as np
import os

obj_dict = {'cracker_box':0,
            'tomato_soup_can':1,
            'sugar_box':2,
            'mustard_bottle':3,
            'potted_meat_can':4,
            'banana':5,
            'bowl':6,
            'mug':7,
            'power_drill':8,
            'scissors':9,
            'chips_can':10,
            'strawberry':11,
            'apple':12,
            'lemon':13,
            'peach':14,
            'pear':15,
            'orange':16,
            'plum':17,
            'knife':18,
            'phillips_screwdriver':19,
            'flat_screwdriver':20,
            'racquetball':21,
            'b_cups':22,
            'd_cups':23,
            'a_toy_airplane':24,
            'c_toy_airplane':25,
            'd_toy_airplane':26,
            'f_toy_airplane':27,
            'h_toy_airplane':28,
            'i_toy_airplane':29,
            'j_toy_airplane':30,
            'k_toy_airplane':31,
            'padlock':32,
            'dragon':33,
            'secret_repair':34,
            'jvr_cleaning_foam':35,
            'dabao_wash_soup':36,
            'nzskincare_mouth_rinse':37,
            'dabao_sod':38,
            'soap_box':39,
            'kispa_cleanser':40,
            'darlie_toothpaste':41,
            'nivea_men_oil_control':42,
            'baoke_marker':43,
            'hosjam':44,
            'pitcher_cap':45,
            'dish':46,
            'white_mouse':47,
            'camel':48,
            'deer':49,
            'zebra':50,
            'large_elephant':51,
            'rhinocero':52,
            'small_elephant':53,
            'monkey':54,
            'giraffe':55,
            'gorilla':56,
            'weiquan':57,
            'darlie_box':58,
            'soap':59,
            'black_mouse':60,
            'dabao_facewash':61,
            'pantene':62,
            'head_shoulders_supreme':63,
            'thera_med':64,
            'dove':65,
            'head_shoulders_care':66,
            'lion':67,
            'coconut_juice_box':68,
            'hippo':69,
            'tape':70,
            'rubiks_cube':71,
            'peeler_cover':72,
            'peeler':73,
            'ice_cube_mould':74,
            'bar_clamp':75,
            'climbing_hold':76,
            'endstop_holder':77,
            'gearbox':78,
            'mount1':79,
            'mount2':80,
            'nozzle':81,
            'part1':82,
            'part3':83,
            'pawn':84,
            'pipe_connector':85,
            'turbine_housing':86,
            'vase':87}

def find_obj_name(obj_dict, model_id):
    object_name = [k for k, v in obj_dict.items() if v == model_id]
    return object_name[0]

if __name__ == "__main__":
    model_path = os.path.abspath('Models')
    model_id = 0
    for i in range(len(obj_dict)):
        obj_name = find_obj_name(obj_dict, model_id)[0]
        os.system('rm Models/{}/{}_target.urdf'.format(str(model_id).zfill(3), obj_name))
        os.system('cp -ifu Models/object_template.urdf Models/{}/{}_target.urdf'.format(str(model_id).zfill(3), obj_name))
        urdf_path = os.path.join(model_path,'{}'.format(str(model_id).zfill(3)),'{}_target.urdf'.format(obj_name))
        obj_path = "D:/code/Dynamic-Grasping/Models/{}/textured.obj".format(str(model_id).zfill(3))

        os.system('sed -i \"s\object_name.obj\{}\g\" {}'.format(str(obj_path), str(urdf_path)))
        os.system('sed -i \"s\object_name\{}\g\" {}'.format(str(obj_name), str(urdf_path)))
        model_id = model_id+1