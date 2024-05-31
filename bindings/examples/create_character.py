import os
import numpy as np
import time
import pyflex
import copy

data = np.load('amp_humanoid_walk.npy', allow_pickle=True).item() # 데이터 로드. @파일명
rotation_data = data['rotation']['arr']
print(rotation_data[0])

# 축의 방향이 다르므로 바꿔주는 함수
# data -> flex
def change_axis(x, y, z):
    return -y,z,x

# 합칠 joint들의 index 받아서 global joint pos 구함
# ex input) [1,3,4,5]
def relative_to_global(relative_index):
    return_pos = copy.deepcopy(relative_joint_positions[0])
    for i in range(len(relative_index)):
        return_pos += relative_joint_positions[relative_index[i]]
    return return_pos

# sphere center로부터 box로 바꿀 때 box 시작점 위치 (box 시작점은 box의 왼쪽 뒷 하단)
def sphere_to_box(center, radius):
    return center[0]-radius, center[1]-radius, center[2]-radius

# fromto로부터 box로 바꿀 때 box 시작점 위치 (box 시작점은 box의 왼쪽 뒷 하단)
def fromto_to_box(joint, start, fromto_unit):
    return joint[0]+start[0]+fromto_unit[0], joint[1]+start[1]+fromto_unit[1], joint[2]+start[2]+fromto_unit[2]

# sphere box로 바꾸고 box scale
def sphere_scale(radius):
    return radius*2, radius*2, radius*2

unit_length = 0.0125
upper_arm_thin = unit_length*7
lower_arm_thin = unit_length*5

upper_leg_thin = unit_length*9
lower_leg_thin = unit_length*7



# 상대적인 위치 이동을 저장한 값 (Flex 좌표축으로 바꿈)
relative_joint_positions = np.array([change_axis(0.0, 0.0, unit_length*80),     # root(pelvis)
                                    change_axis(0.0, 0.0, unit_length*19),      # torso
                                    change_axis(0.0, 0.0, unit_length*18),      # head
                                    
                                    change_axis(-unit_length*2, -unit_length*14, unit_length*18), # right_upper_arm
                                    change_axis(0.0, 0.0, -unit_length*22),                       # right_lower_arm
                                    change_axis(0.0, 0.0, -unit_length*20),                       # right_hand
                                    
                                    change_axis(-unit_length*2, unit_length*14, unit_length*18),  # left_upper_arm
                                    change_axis(0.0, 0.0, -unit_length*22),                       # left_lower_arm
                                    change_axis(0.0, 0.0, -unit_length*20),                       # left_hand
                                    
                                    change_axis(0.0, -unit_length*7, 0.0),            # right_thigh
                                    change_axis(0.0, 0.0, -unit_length*34),           # right_shin
                                    change_axis(0.0, 0.0, -unit_length*32),           # right_foot
                                    
                                    change_axis(0.0, unit_length*7, 0.0),            # left_thigh
                                    change_axis(0.0, 0.0, -unit_length*34),           # left_shin
                                    change_axis(0.0, 0.0, -unit_length*32)            # left_foot
                                    ])

# 순서는 relative와 같음 (Flex 좌표축임)
global_joint_positions = np.array([relative_joint_positions[0],
                                                    
                                   relative_to_global([1]),
                                   relative_to_global([1,2]),

                                   relative_to_global([1,3]),
                                   relative_to_global([1,3,4]),
                                   relative_to_global([1,3,4,5]),

                                   relative_to_global([1,6]),
                                   relative_to_global([1,6,7]),
                                   relative_to_global([1,6,7,8]),

                                   relative_to_global([9]),
                                   relative_to_global([9,10]),
                                   relative_to_global([9,10,11]),

                                   relative_to_global([12]),
                                   relative_to_global([12,13]),
                                   relative_to_global([12,13,14]),
])

geom_start_pos  = np.array([# pelvis
                            sphere_to_box(global_joint_positions[0] + np.array([0, unit_length*6, 0]), unit_length*7.5),     
                            sphere_to_box(global_joint_positions[0] + np.array([0, unit_length*16, 0]), unit_length*5.5),
                            sphere_to_box(global_joint_positions[1] + np.array([0, unit_length*9, 0]), unit_length*9.5) + np.array([-unit_length*5, 0.0, 0.0]),
                            
                            # neck
                            global_joint_positions[2] + np.array([-unit_length*2.5, -unit_length*0.5, -unit_length*2.5]),
                            
                            # head
                            sphere_to_box(global_joint_positions[2] + np.array([0, unit_length*11,0]), unit_length*7.5),

                            # right arm
                            fromto_to_box(global_joint_positions[3], np.array([0, -unit_length*15.5, 0]), np.array([-upper_arm_thin/2, -upper_arm_thin, -upper_arm_thin/2])),
                            fromto_to_box(global_joint_positions[4], np.array([0, -unit_length*14.5, 0]), np.array([-lower_arm_thin/2, -lower_arm_thin, -lower_arm_thin/2])),
                            sphere_to_box(global_joint_positions[5], unit_length*3.5),
                            
                            # left arm
                            fromto_to_box(global_joint_positions[6], np.array([0, -unit_length*15.5, 0]), np.array([-upper_arm_thin/2, -upper_arm_thin, -upper_arm_thin/2])),
                            fromto_to_box(global_joint_positions[7], np.array([0, -unit_length*14.5, 0]), np.array([-lower_arm_thin/2, -lower_arm_thin, -lower_arm_thin/2])),
                            sphere_to_box(global_joint_positions[8], unit_length*3.5),

                            # right leg
                            fromto_to_box(global_joint_positions[9], np.array([0, -unit_length*24.5, 0]), np.array([-upper_leg_thin/2, -upper_leg_thin, -upper_leg_thin/2])),
                            fromto_to_box(global_joint_positions[10], np.array([0, -unit_length*25.5, 0]), np.array([-lower_leg_thin/2, -lower_leg_thin, -lower_leg_thin/2])),
                            global_joint_positions[11] + np.array([-lower_leg_thin/2, -unit_length*3.5, -lower_leg_thin/2]),

                            # left leg
                            fromto_to_box(global_joint_positions[12], np.array([0, -unit_length*24.5, 0]), np.array([-upper_leg_thin/2, -upper_leg_thin, -upper_leg_thin/2])),
                            fromto_to_box(global_joint_positions[13], np.array([0, -unit_length*25.5, 0]), np.array([-lower_leg_thin/2, -lower_leg_thin, -lower_leg_thin/2])),
                            global_joint_positions[14] + np.array([-lower_leg_thin/2, -unit_length*3.5, -lower_leg_thin/2]),
                            ])

geom_scales     = np.array([[unit_length*15, unit_length*15, unit_length*15],
                            [unit_length*11, unit_length*11, unit_length*11],
                            [unit_length*19+unit_length*10, unit_length*19, unit_length*19],

                            # neck
                            [unit_length*5, unit_length*5, unit_length*5],
                            
                            # head
                            [unit_length*15, unit_length*15, unit_length*15],

                            # right arm
                            [upper_arm_thin, unit_length*23, upper_arm_thin],
                            [lower_arm_thin, unit_length*20, lower_arm_thin],
                            [unit_length*7, unit_length*7, unit_length*7],

                            # left arm
                            [upper_arm_thin, unit_length*23, upper_arm_thin],
                            [lower_arm_thin, unit_length*20, lower_arm_thin],
                            [unit_length*7, unit_length*7, unit_length*7],

                            # right leg
                            [upper_leg_thin, unit_length*34, upper_leg_thin],
                            [lower_leg_thin, unit_length*33, lower_leg_thin],
                            [lower_leg_thin, unit_length*4, unit_length*14],

                            # left leg
                            [upper_leg_thin, unit_length*34, upper_leg_thin],
                            [lower_leg_thin, unit_length*33, lower_leg_thin],
                            [lower_leg_thin, unit_length*4, unit_length*14]
])

density         = np.array([2226,
                            2226,
                            1794,

                            # head, neck
                            1081,
                            1081,

                            # right arm
                            982,
                            1056,
                            1865,

                            # left arm
                            982,
                            1056,
                            1865,

                            # right leg
                            1269,
                            1014,
                            1141,

                            # left leg
                            1269,
                            1014,
                            1141,
                            ])

geom_start_pos = geom_start_pos*4
geom_scales = geom_scales*4

draw_spring = np.array([0.0])

np.set_printoptions(suppress=True)
scene_params= np.concatenate((geom_start_pos.reshape(-1), geom_scales.reshape(-1), density, draw_spring))
print(scene_params)


pyflex.init()
time.sleep(3)
pyflex.set_scene(1,scene_params,0)

for i in range(1000):
    pyflex.step()

pyflex.clean()