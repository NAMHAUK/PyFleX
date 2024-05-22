import gymnasium as gym
import pyflex
import numpy as np
import math
from gymnasium import spaces

center_index = 12
tartget_x = 5.0

# 밑 판, stick parameters
scene_params = np.zeros(9)
scene_params[0] = 1

# 밑 판 위치
scene_params[1] = 0.0
scene_params[2] = 0.0
scene_params[3] = 0.0

# stick 위치
scene_params[4] = 0.1
scene_params[5] = 0.0
scene_params[6] = 0.1

# springStiffness(stick 밑과 연결, 위와 연결)
scene_params[7] = 1.0
scene_params[8] = 5.0


# 판에서 unit length
unit_length = 0.025

class MyEnv(gym.Env):
    """Custom Environment that follows gym interface."""

    metadata = {"render_modes": ["human"], "render_fps": 30}
    
    def __init__(self, env_config = None):
        super().__init__()
        print("super init")
        pyflex.init()
        print("pyflex init 완료")
        pyflex.set_scene(3, scene_params, 0)
        print("set scene 완료")
        self.max_action = 10
        min_action = -self.max_action

        # x축 기준 회전, z축 기준 회전, 횟수 주기
        # self.action_space = spaces.Box(
        #     np.array([self.min_action, self.min_action, 0.0]),
        #     np.array([self.max_action, self.max_action, 1.0]),
        #     dtype=np.float32
        # )
        self.action_space = spaces.MultiDiscrete([self.max_action*2+1 , self.max_action*2+1, 4])
        # 누적 x축 기준 회전 각도, 누적 z축 기준 회전 각도, 물체의 중심점 위치(x,y,z)
        self.observation_space = spaces.Box(
            np.array([-90.0, -90.0, -5.0,  0.0, -5.0]),
            np.array([ 90.0 , 90.0, -5.0, -5.0, -5.0]),
            dtype=np.float32
        )
        
        self.observation = np.array([0.0, 0.0, 0.1, 0.0, 0.1], dtype=np.float32)
        
        self.total_reward = 0
        self.total_step_count = 0
        self.nan_count = 0
        self.episode_count = 0
        self.y_count =0
        print("space 설정 완료")

    def reset(self, seed=None, options=None):
        # 회전한 각도를 모두 0으로 만들어 stick을 중심으로 오게 함
        # 물체 위치를 0, 0, 0으로 돌아오게 함
        self.observation = np.array([0.0, 0.0, 0.1, 0.0, 0.1], dtype=np.float32)
        
        ####
        print("total_reward = ",self.total_reward, " episode count = ", self.episode_count, " total_count=",self.total_step_count)
        self.total_reward = 0
        self.nan_count = 0
        self.episode_count = 0 
        self.y_count = 0
        ####
        
        pyflex.set_scene(3, scene_params, 0)
        pyflex.step()
        
        return self.observation, {} 
    
    def step(self, action):
        # # action에 따라 각도 update
        # self.observation[0] += (action[0]-self.max_action)
        # self.observation[1] += (action[1]-self.max_action)
        self.observation[0] = action[0]
        self.observation[1] = action[1]
        # # 매번 action 받고 해당 각도로 움직인 후 다시 중심으로 돌림
        # self.observation[0] = action[0] - self.max_action
        # self.observation[1] = action[1] - self.max_action
        
        # update한 angle에 따라 spring 길이를 조절하여 stick을 움직이고 1 step 진행
        pyflex.handle_spring(cal_spring_lengths(get_stick_position(self.observation[0], self.observation[1])))
        pyflex.step()
        # for i in range(action[2]+1):
        #     pyflex.step()
        # pyflex.handle_spring(stick_move_center())
        # pyflex.step()
        
        # 현재 밑 판의 center값 저장
        cur_step_rigid_positions = pyflex.get_positions().reshape(-1, 4)
        
        last_position_x = self.observation[2]
        last_position_z = self.observation[4]
        self.observation[2], self.observation[3], self.observation[4] = self.get_center_positions(cur_step_rigid_positions, center_index)
        
        
        # reward는 일단 x position 값만(변위로 해야함)
        reward = (self.observation[2]-last_position_x) * 100
        reward -= abs((self.observation[4]-last_position_z) )*10
        
        # 목표 x 값에 도달했으면 terminated
        terminated = False
        trucated = False
        if(self.observation[2]>=tartget_x):
            terminated = True
            reward += 1000
        if(self.episode_count>1000):
            trucated = True
        
        # stick top의 y값 얻음
        x,y,z = self.get_center_positions(cur_step_rigid_positions, 29)
        # 엎어짐을 판단 연속적이면 y_count ++
        if(y<unit_length*2):
            self.y_count += 1
        else :
            self.y_count = 0
            
        # 연속으로 있으면 엎어진거로 판단
        if(self.y_count >= 3):
            trucated = True
            reward -=100
        ###
        self.total_reward += reward
        self.total_step_count +=1
        self.episode_count +=1
        ###
        return (self.observation, reward, terminated, trucated, {})

    def close(self):
        pyflex.close()
        # rigid_positions에서 밑 판의 center position만 얻음
    
    def get_center_positions(self, rigid_positions, center_index):
        # y 위치 평균 구해서 reward 계산
        rigid_positions_nan = np.isnan(rigid_positions)
        # 만약 center_index의 값이 nan이면 이전 observation 값을 return
        # 순서대로 x, y, z
        center_position_x = rigid_positions[center_index][0]
        center_position_y = rigid_positions[center_index][1]
        center_position_z = rigid_positions[center_index][2]
        
        if(rigid_positions_nan[center_index][0]==True):
            self.nan_count += 1
            center_position_x = self.observation[2]
        if(rigid_positions_nan[center_index][1]==True):
            self.nan_count += 1
            center_position_y = self.observation[3]
        if(rigid_positions_nan[center_index][2]==True):
            self.nan_count += 1
            center_position_z = self.observation[4]
            
        return center_position_x, center_position_y, center_position_z
        
    


def cal_distance(x1,y1,z1, x2,y2,z2):
    return math.sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)

# angle 값을 이용하여 판에서 stick의 상대적인 위치를 구함
def get_stick_position(angle_a, angle_r):
    roll = np.radians(angle_a)
    yaw  = np.radians(angle_r)
    # print(roll, yaw)
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw), np.cos(yaw), 0],
                   [0, 0, 1]])
    R = np.dot(Rz, Rx)

    # stick 위쪽 끝 부분을 회전시킨 끝의 위치를 구함
    stick_top = np.array([0, 8*unit_length, 0]).T
    stick_position = np.dot(R, stick_top)
    
    # z축 방향이 반대이므로 - 부호 붙임
    stick_position[2] = - stick_position[2]

    # stick position 판의 중심으로 이동
    stick_position = stick_position+ np.array([5*unit_length, unit_length, 5*unit_length]).T
    return stick_position

# 판에서 stick의 상대적인 위치를 통해 판의 각 부분과 길이(spring의 길이)를 구함
def cal_spring_lengths(stick_position):
    spring_lengths = np.zeros(8)
    # position 값을 통해 각 spring의 length를 구하고 return
    stick_pos_x = stick_position[0]
    stick_pos_y = stick_position[1]
    stick_pos_z = stick_position[2]
    
    spring_lengths[0] = cal_distance(stick_pos_x, stick_pos_y, stick_pos_z, unit_length  , unit_length, unit_length)
    spring_lengths[1] = cal_distance(stick_pos_x, stick_pos_y, stick_pos_z, unit_length  , unit_length, 5*unit_length)
    spring_lengths[2] = cal_distance(stick_pos_x, stick_pos_y, stick_pos_z, unit_length  , unit_length, 9*unit_length)
    spring_lengths[3] = cal_distance(stick_pos_x, stick_pos_y, stick_pos_z, 5*unit_length, unit_length, 9*unit_length)
    spring_lengths[4] = cal_distance(stick_pos_x, stick_pos_y, stick_pos_z, 9*unit_length, unit_length, 9*unit_length)
    spring_lengths[5] = cal_distance(stick_pos_x, stick_pos_y, stick_pos_z, 9*unit_length, unit_length, 5*unit_length)
    spring_lengths[6] = cal_distance(stick_pos_x, stick_pos_y, stick_pos_z, 9*unit_length, unit_length, unit_length)
    spring_lengths[7] = cal_distance(stick_pos_x, stick_pos_y, stick_pos_z, 5*unit_length, unit_length, unit_length)
        
    # spring_lengths[0] = cal_distance(stick_pos_x, stick_pos_y, stick_pos_z, unit_length  , unit_length, 9*unit_length)
    # spring_lengths[1] = cal_distance(stick_pos_x, stick_pos_y, stick_pos_z, unit_length  , unit_length, 5*unit_length)
    # spring_lengths[2] = cal_distance(stick_pos_x, stick_pos_y, stick_pos_z, unit_length  , unit_length, unit_length)
    # spring_lengths[3] = cal_distance(stick_pos_x, stick_pos_y, stick_pos_z, 5*unit_length, unit_length, unit_length)
    # spring_lengths[4] = cal_distance(stick_pos_x, stick_pos_y, stick_pos_z, 9*unit_length, unit_length, unit_length)
    # spring_lengths[5] = cal_distance(stick_pos_x, stick_pos_y, stick_pos_z, 9*unit_length, unit_length, 5*unit_length)
    # spring_lengths[6] = cal_distance(stick_pos_x, stick_pos_y, stick_pos_z, 9*unit_length, unit_length, 9*unit_length)
    # spring_lengths[7] = cal_distance(stick_pos_x, stick_pos_y, stick_pos_z, 5*unit_length, unit_length, 9*unit_length)
    
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