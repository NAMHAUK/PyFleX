import math
from typing import Optional
import pyflex
import numpy as np

import gymnasium as gym
from gymnasium import spaces
from gymnasium.envs.classic_control import utils
from gymnasium.error import DependencyNotInstalled

# env state index
X_BOX = 0
V_BOX = 1
X_BOX_LAST = 2


# box params
border = 0.025
height = 1.0
x_center = 0.0

# fluid params:
sx_f = 10
sy_f = 10
sz_f = 3
px_f = x_center - (sx_f - 1) / 2. * 0.055
py_f = 0.055 / 2. + border + 0.01
pz_f = 0. - (sz_f - 1) / 2. * 0.055

# rigid params:
sx_r = 0.2
sy_r = 0.2
sz_r = 0.15
px_r = x_center - sx_r / 2.
py_r = py_f + sy_f * 0.052
pz_r = -sz_r / 2.

# box의 x축, z축 길이
box_dis_x = sx_f * 0.055 + 0.1
box_dis_z = 0.125

cohesion = 0.2
suface = 0.001

scene_params = np.array([
    px_f, py_f, pz_f, sx_f, sy_f, sz_f,
    px_r, py_r, pz_r, sx_r, sy_r, sz_r,
    cohesion, suface,
    box_dis_x, box_dis_z])

# 매 step마다 action을 하고 ICE의 위치를 계산해서, ICE의 위치가 높을 수록 큰 reward를 주는 환경
class Continuous_Env(gym.Env):
    def __init__(self, render_mode: Optional[str] = None):
        pyflex.init()
        print("시작")
        pyflex.set_scene(8, scene_params, 0)
        print("set_scene은 됨")
        # 최대, 최소 힘
        self.min_action = -0.3
        self.max_action = 0.3

        # box 크기에 대한 값
        self.border    = border
        self.height    = height
        self.box_dis_x = box_dis_x
        self.box_dis_z = box_dis_z

        # rigid 관련
        self.n_rigidPositions = pyflex.get_n_rigidPositions()
        print("rigid 개수 얻는 것 됨")
        # parameter
        self.dt         = 1. / 60.
        self.dim_rigid  = 3
        self.dim_shape_state = 14
        self.count      = 0

        boxes = calc_box_init(self.box_dis_x, self.box_dis_z, self.border, self.height)
        
        for j in range(len(boxes)):
            halfEdge = boxes[j][0]
            center = boxes[j][1]
            quat = boxes[j][2]
            pyflex.add_box(halfEdge, center, quat) 
        print("add box는 됨")
        # action이 작용할 수 있는 범위 설정
        # box를 좌우로 움직이는 속도
        self.action_space = spaces.Box(
            np.array([self.min_action]),
            np.array([self.max_action]),
            dtype=np.float32
        )
        print("action space 됨")
        # box의 위치
        # x_box, v_box, x_box_last
        self.state = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.observation_space = spaces.Box(
            np.array([-1.0,-1.0,-1.0]),
            np.array([1.0, 1.0, 1.0]),
            dtype=np.float32
        )

    # 이번 step의 action을 받아서 실행해서 box의 위치를 update
    # box의 움직임으로 인한 shape_state 변화 return
    # pyflex가 여기서 가능하다면, position을 받을 필요도 없고, 현재 step의 마지막에 position 평가하는게 더 좋음
    def step(self, action):
        self.count +=1
        # action으로 box위치 update, shape_state update
        self.state[X_BOX_LAST] = self.state[X_BOX]
        self.state[X_BOX]     += self.state[V_BOX] * self.dt
        self.state[V_BOX]     += action - self.state[X_BOX] * 0.1
        # self.state[V_BOX]   += action

        shape_states = calc_shape_states(self.state[X_BOX], self.state[X_BOX_LAST], self.box_dis_x, self.box_dis_z, self.border, self.height)
        pyflex.set_shape_states(shape_states)
        
        # rigid position구하고, y위치로 reward(y값 평균) 계산
        cur_step_rigid_positions    = pyflex.get_rigidGlobalPositions().reshape(-1, self.dim_rigid)
        avg_rigid_y_positions       = self.calc_y_positions(cur_step_rigid_positions, self.n_rigidPositions)


        print("avg_rigid_y", avg_rigid_y_positions)
        # rigid 물체가 box를 나간다면 종료
        #(평균 값이므로 0.1 뺐는데 이 값은 해보고 조정해야 할듯
        #(아니면 아예 일정 횟수가 지나면 종료해서 reward 가장 높게만들어야 할듯)
        terminated = bool(
            avg_rigid_y_positions >= self.height - 0.1
        )

        # 높게 있을 수록(빠져 나가려 할 수록) reward가 높음
        # goal에 도달 시 보상을 추가로 더 줌(얼마나 줄지는 해봐야 알듯)
        reward = avg_rigid_y_positions

        if terminated:
            reward += 50

        # step 위치가 여기가 맞을까
        pyflex.step()
        return self.state, reward, terminated, False, {}
    
    # 이번 step으로 실행된 결과로인한 rigid position들로 평가함.
    def calc_y_positions(self, rigid_positions, n_rigidPositions):
        
        # y 위치 평균 구해서 reward 계산
        total_y = 0.0
        avg_rigid_y  = 0.0
        rigid_count = n_rigidPositions
        rigid_positions_nan = np.isnan(rigid_positions)
        real_rigid_count = 0
        for i in range(rigid_count):
            if(rigid_positions_nan[i][1]):
                print("nan")
            else:
                real_rigid_count += 1
                total_y += rigid_positions[i][1]

        if(rigid_count !=0):
            avg_rigid_y = total_y /real_rigid_count
        else:
            print("엥")
        return avg_rigid_y
    
    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        # box의 정보 초기화 (가운데에 다시 놓음)
        pyflex.set_scene(8, scene_params, 0)
        boxes = calc_box_init(self.box_dis_x, self.box_dis_z, self.border, self.height)
        for j in range(len(boxes)):
            halfEdge = boxes[j][0]
            center = boxes[j][1]
            quat = boxes[j][2]
            pyflex.add_box(halfEdge, center, quat)   

        self.state = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.count      = 0
        return self.state, {}


def calc_box_init(dis_x, dis_z, border, height):
    center = np.array([0., 0., 0.])
    quat = np.array([1., 0., 0., 0.])
    boxes = []

    # floor
    halfEdge = np.array([dis_x/2., border/2., dis_z/2.])
    boxes.append([halfEdge, center, quat])

    # left wall
    halfEdge = np.array([border/2., (height+border)/2., dis_z/2.])
    boxes.append([halfEdge, center, quat])

    # right wall
    boxes.append([halfEdge, center, quat])

    # back wall
    halfEdge = np.array([(dis_x+border*2)/2., (height+border)/2., border/2.])
    boxes.append([halfEdge, center, quat])

    # front wall
    boxes.append([halfEdge, center, quat])

    return boxes


def calc_shape_states(x_curr, x_last, dis_x, dis_z, border, height):
    quat = np.array([1., 0., 0., 0.])

    states = np.zeros((5, 14))

    states[0, :3] = np.array([x_curr, border/2., 0.])
    states[0, 3:6] = np.array([x_last, border/2., 0.])

    states[1, :3] = np.array([x_curr-(dis_x+border)/2., (height+border)/2., 0.])
    states[1, 3:6] = np.array([x_last-(dis_x+border)/2., (height+border)/2., 0.])

    states[2, :3] = np.array([x_curr+(dis_x+border)/2., (height+border)/2., 0.])
    states[2, 3:6] = np.array([x_last+(dis_x+border)/2., (height+border)/2., 0.])

    states[3, :3] = np.array([x_curr, (height+border)/2., -(dis_z+border)/2.])
    states[3, 3:6] = np.array([x_last, (height+border)/2., -(dis_z+border)/2.])

    states[4, :3] = np.array([x_curr, (height+border)/2., (dis_z+border)/2.])
    states[4, 3:6] = np.array([x_last, (height+border)/2., (dis_z+border)/2.])

    states[:, 6:10] = quat
    states[:, 10:] = quat

    return states