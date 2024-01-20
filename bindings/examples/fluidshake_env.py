import math
from typing import Optional
import pyflex
import numpy as np

import gymnasium as gym
from gymnasium import spaces
from gymnasium.envs.classic_control import utils
from gymnasium.error import DependencyNotInstalled


# 매 step마다 ICE의 위치를 받아서, ICE의 위치가 높을 수록 큰 reward를 주는 환경
class Continuous_Env(gym.Env):
    metadata = {
        "render_modes": ["human", "rgb_array"],
        "render_fps": 30,
    }

    def __init__(self, border, height, box_dis_x, box_dis_z, render_mode: Optional[str] = None):
        pyflex.init()
        # 최대, 최소 힘
        self.min_action = -0.3
        self.max_action = 0.3

        # box 크기에 대한 값
        self.dim_shape_state = 14
        self.border    = border
        self.height    = height
        self.box_dis_x = box_dis_x
        self.box_dis_z = box_dis_z
        self.boxes     = []
        
        # box의 위치
        self.x_box_last = 0.0
        self.x_box      = 0.0
        self.v_box      = 0.0

        # parameter
        self.dt         = 1. / 60.

        # self.state 지정해주기
        # 매 step마다 box의 shape_state 계산해서 저장.
        self.state = np.zeros((5, self.dim_shape_state))

        self.action_space = spaces.Box(
            low=self.min_action, high=self.max_action
        )

    # 이번 step의 action을 받아서 실행해서 box의 위치를 update
    # box의 움직임으로 인한 shape_state 변화 return
    # pyflex가 여기서 가능하다면, position을 받을 필요도 없고, 현재 step의 마지막에 position 평가하는게 더 좋음
    def step(self, action):

        # action으로 box위치 update, shape_state update
        self.x_box_last =  self.x_box
        self.x_box      += self.v_box * self.dt
        self.v_box      += action - self.x_box * 0.1

        self.state      = calc_shape_states(self.x_box, self.x_box_last, self.box_dis_x, self.box_dis_z,self.border, self.height)

        return self.state
    
    # 이번 step으로 실행된 결과로인한 rigid position들로 평가함.
    def calc_reward(self, rigid_positions, n_rigidPositions):
        
        # y 위치 평균 구해서 reward 계산
        total_y = 0.0
        reward  = 0.0
        rigid_count = n_rigidPositions
        rigid_positions_nan = np.isnan(rigid_positions)
        for i in range(rigid_count):
            if(rigid_positions_nan[i][1]):
                print("nan")
            else:
                total_y += rigid_positions[i][1]

        if(rigid_count !=0):
            reward = total_y /rigid_count
        else:
            print("엥")
    
        reward = reward*reward
        return reward
    
    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        super().reset(seed=seed)
        # box의 정보 초기화 (가운데에 다시 놓음)
        # pyflex 사용되면, 여기서 set_scene 까지
        self.boxes = calc_box_init(self.box_dis_x, self.box_dis_z, self.border, self.height)
        self.state = np.zeros((5, self.dim_shape_state))
        self.x_box_last = 0.0
        self.x_box      = 0.0
        self.v_box      = 0.0

        return self.boxes, self.state, {}


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