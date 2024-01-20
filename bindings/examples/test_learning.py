import os
import numpy as np
import pyflex
import time

import scipy.spatial as spatial
from sklearn.decomposition import PCA

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

import gymnasium as gym
import fluidshake_env as ENV


# 1초에 60번 실행
# 총 30000번 실행, 한 episode는 600번(10초)실행
dt = 1. / 60.
time_step = 30000
one_episode_step = 600

dim_position = 4
dim_rigid    = 3
dim_velocity = 3
dim_shape_state = 14

border = 0.025
height = 1.0

def rand_float(lo, hi):
    return np.random.rand() * (hi - lo) + lo
### set scene

# fluid params:
# px_f: [0.0, 0.3]
# py_f: [0.1, 0.2]
# pz_f: [0.0, 0.3]
# sx_f: [8, 12]
# sy_f: [15, 20]
# sz_f: 3

# rigid params:
# px_r: []
# py_r: []
# pz_r: []
# sx_r: []
# sy_r: []
# sz_r: []

sx_f = 10
sy_f = 10
sz_f = 3
x_center = 0.0
px_f = x_center - (sx_f - 1) / 2. * 0.055
py_f = 0.055 / 2. + border + 0.01
pz_f = 0. - (sz_f - 1) / 2. * 0.055
box_dis_x = sx_f * 0.055 + 0.1
box_dis_z = 0.125

sx_r = 0.2
sy_r = 0.2
sz_r = 0.15
px_r = x_center - sx_r / 2.
py_r = py_f + sy_f * 0.052
pz_r = -sz_r / 2.

scene_params = np.array([
    px_f, py_f, pz_f, sx_f, sy_f, sz_f,
    px_r, py_r, pz_r, sx_r, sy_r, sz_r,
    box_dis_x, box_dis_z])

# -------------------------------------------------------------------
pyflex.init()
pyflex.set_scene(8, scene_params, 0)

env = ENV.Continuous_Env(border, height, box_dis_x, box_dis_z)
boxes, shape_states, info = env.reset()

for i in range(len(boxes)):
    halfEdge = boxes[i][0]
    center = boxes[i][1]
    quat = boxes[i][2]
    pyflex.add_box(halfEdge, center, quat)

n_particles = pyflex.get_n_particles()
n_shapes = pyflex.get_n_shapes()
n_rigidPositions = pyflex.get_n_rigidPositions()

# -------------------------------------------------------------------
cur_episode_step = 0            # 현재 episode의 reward
cur_episode_reward = 0          # 현재 episode의 reward 합을 저장
cur_episode_positions     = np.zeros((one_episode_step, n_particles, dim_position))  # 현재 episode에서 particle들의 position 모두 저장
cur_episode_shape_states  = np.zeros((one_episode_step, n_shapes, dim_shape_state))  # 현재 episode에서 particle들의 position 모두 저장
cur_step_rigid_positions  = np.zeros((n_rigidPositions, dim_rigid))                  # 현재 step의 rigid particle들의 position 모두 저장

best_episode_positions    = np.zeros((one_episode_step, n_particles, dim_position))  # best episode에서 particle들의 position 모두 저장
best_episode_shape_states = np.zeros((one_episode_step, n_shapes, dim_shape_state))  # best episode에서 shape_states 저장
best_episode_reward = -1        # 모든 episode 중에서 가장 reward가 컸던 episode의 reward값 저장

x_box = x_center
v_box = 0
for i in range(time_step):
    #action = env.action_space.sample()
    action = rand_float(-0.3, 0.3)
    shape_states_ = env.step(action)
    pyflex.set_shape_states(shape_states_)
    
    # box 위치 변경에 따른 값들 계산
    cur_episode_positions[cur_episode_step]    = pyflex.get_positions().reshape(-1, dim_position)
    cur_episode_shape_states[cur_episode_step] = pyflex.get_shape_states().reshape(-1, dim_shape_state)

    # rigid position구하고, y위치로 reward 계산
    cur_step_rigid_positions    = pyflex.get_rigidGlobalPositions().reshape(-1,dim_rigid)
    cur_episode_reward         += env.calc_reward(cur_step_rigid_positions, n_rigidPositions)

    cur_episode_step += 1
    pyflex.step()
    # 한 episode 종료 시(600번=10초)
    # 1. 환경 reset
    # 2. 현재 episode가 best인지 확인 후, best이면 best값들 갱신
    # 3. 다음 episode를 위해 cur_~ 값들 초기화
    # 4. pyflex scene 다시 실행하여 장면 초기화
    # 5. 장면 갱신을 위해, 관련 값들 다시 초기화
    if(((i+1) % one_episode_step) == 0):
        boxes, shape_states, info = env.reset()
        print("episode", i+1," end, reward :", cur_episode_reward)

        if ( best_episode_reward < cur_episode_reward):
            best_episode_reward       = cur_episode_reward
            best_episode_positions    = cur_episode_positions
            best_episode_shape_states = cur_episode_shape_states
        
        cur_episode_positions     = [] 
        cur_episode_shape_states  = []
        cur_episode_step          = 0
        cur_episode_reward        = 0

        pyflex.set_scene(8, scene_params, 0)

        for j in range(len(boxes)):
            halfEdge = boxes[j][0]
            center = boxes[j][1]
            quat = boxes[j][2]
            pyflex.add_box(halfEdge, center, quat)   
         
        n_particles = pyflex.get_n_particles()
        n_shapes = pyflex.get_n_shapes()
        n_rigidPositions = pyflex.get_n_rigidPositions()

        cur_episode_positions = np.zeros((one_episode_step, n_particles, dim_position))
        cur_episode_shape_states = np.zeros((one_episode_step, n_shapes, dim_shape_state)) 


# 가장 잘 학습된 best episode로 실행한 장면을 보여줌
pyflex.set_scene(8, scene_params, 0)
for i in range(len(boxes)-1):
    halfEdge = boxes[i][0]
    center = boxes[i][1]
    quat = boxes[i][2]
    pyflex.add_box(halfEdge, center, quat)

des_dir = 'test_FluidIceShake'
os.system('mkdir -p ' + des_dir)

for i in range(one_episode_step):
    pyflex.set_positions(best_episode_positions[i])
    pyflex.set_shape_states(best_episode_shape_states[i, :-1])

    pyflex.render(capture=1, path=os.path.join(des_dir, 'render_%d.tga' % i))

pyflex.clean()
