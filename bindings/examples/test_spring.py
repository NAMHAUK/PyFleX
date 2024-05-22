import os
import numpy as np
import pyflex
import time
import torch
import math

des_dir = 'test_Spring'
os.system('mkdir -p ' + des_dir)

unit_length = 0.025
def cal_distance(x1,y1,z1, x2,y2,z2):
    return math.sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)

# 일단 왼쪽으로 45도 움직이는 것만 생각했는데, 값을 바꿀 수도 있을듯
# rigid_fall.h에서 stick만 무게 바꾸는 것도 ㄱㅊ을듯
def stick_move_left():
    spring_lengths = np.zeros(8)
    
    stick_pos_x = 5*unit_length - 4*math.sqrt(2)*unit_length
    stick_pos_y = 4*math.sqrt(2)*unit_length + unit_length
    stick_pos_z = 5*unit_length

    # 왼쪽 앞부터 시계방향
    spring_lengths[0] = cal_distance(stick_pos_x, stick_pos_y, stick_pos_z, unit_length  , unit_length, unit_length)
    spring_lengths[1] = cal_distance(stick_pos_x, stick_pos_y, stick_pos_z, unit_length  , unit_length, 5*unit_length)
    spring_lengths[2] = cal_distance(stick_pos_x, stick_pos_y, stick_pos_z, unit_length  , unit_length, 9*unit_length)
    spring_lengths[3] = cal_distance(stick_pos_x, stick_pos_y, stick_pos_z, 5*unit_length, unit_length, 9*unit_length)
    spring_lengths[4] = cal_distance(stick_pos_x, stick_pos_y, stick_pos_z, 9*unit_length, unit_length, 9*unit_length)
    spring_lengths[5] = cal_distance(stick_pos_x, stick_pos_y, stick_pos_z, 9*unit_length, unit_length, 5*unit_length)
    spring_lengths[6] = cal_distance(stick_pos_x, stick_pos_y, stick_pos_z, 9*unit_length, unit_length, unit_length)
    spring_lengths[7] = cal_distance(stick_pos_x, stick_pos_y, stick_pos_z, 5*unit_length, unit_length, unit_length)

    return spring_lengths

def stick_move_center():
    spring_lengths = np.zeros(8)
    stick_pos_x = 5*unit_length
    stick_pos_y = 9*unit_length
    stick_pos_z = 5*unit_length

    # 왼쪽 앞부터 시계방향
    spring_lengths[0] = cal_distance(stick_pos_x, stick_pos_y, stick_pos_z, unit_length  , unit_length, unit_length)
    spring_lengths[1] = cal_distance(stick_pos_x, stick_pos_y, stick_pos_z, unit_length  , unit_length, 5*unit_length)
    spring_lengths[2] = cal_distance(stick_pos_x, stick_pos_y, stick_pos_z, unit_length  , unit_length, 9*unit_length)
    spring_lengths[3] = cal_distance(stick_pos_x, stick_pos_y, stick_pos_z, 5*unit_length, unit_length, 9*unit_length)
    spring_lengths[4] = cal_distance(stick_pos_x, stick_pos_y, stick_pos_z, 9*unit_length, unit_length, 9*unit_length)
    spring_lengths[5] = cal_distance(stick_pos_x, stick_pos_y, stick_pos_z, 9*unit_length, unit_length, 5*unit_length)
    spring_lengths[6] = cal_distance(stick_pos_x, stick_pos_y, stick_pos_z, 9*unit_length, unit_length, unit_length)
    spring_lengths[7] = cal_distance(stick_pos_x, stick_pos_y, stick_pos_z, 5*unit_length, unit_length, unit_length)
    
    return spring_lengths

pyflex.init()
n_instance = 2

scene_params = np.zeros(12)
scene_params[0] = n_instance

scene_params[1] = 0.0
scene_params[2] = 0.0
scene_params[3] = 0.0

scene_params[4] = 0.1
scene_params[5] = 0.0
scene_params[6] = 0.1

# springStiffness
scene_params[7] = 1.0
scene_params[8] = 1.0
pyflex.set_scene(3, scene_params, 0)

spring_lengths_left = stick_move_left()
spring_lengths_center = stick_move_center()

# stick이 움직이는 time은 안넘어지게 값을 바꿔가면서 해보아야 할듯
chk = 0
stick_move_time = 5
for i in range(1000):
    pyflex.step()
    if (i % stick_move_time)==0:
        if(chk == 0): # chk == 0 이면 stick을 center로
            pyflex.handle_spring(spring_lengths_center)
            chk = 1
        else: # chk == 1 이면 stick을 left로
            pyflex.handle_spring(spring_lengths_left)
            chk = 0
    
pyflex.clean()
