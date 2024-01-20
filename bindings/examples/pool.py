import os
import numpy as np
import pyflex
import time

import scipy.spatial as spatial
from sklearn.decomposition import PCA

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


dt = 1. / 60.

time_step = 900
time_step_rest = 30
dim_position = 4
dim_velocity = 3
dim_shape_state = 14

border = 0.025
box_dis_y = 1.0

floor_move = 0

def calc_box_init(cent, dis_x, dis_z):
    center = cent
    # center = np.array([0., 0., 0.])
    quat = np.array([1., 0., 0., 0.])
    boxes = []

    # floor
    halfEdge = np.array([dis_x/2., border/2., dis_z/2.])
    boxes.append([halfEdge, center, quat])

    # left wall
    halfEdge = np.array([border/2., (box_dis_y+border)/2., dis_z/2.])
    boxes.append([halfEdge, center, quat])

    # right wall
    boxes.append([halfEdge, center, quat])

    # back wall
    halfEdge = np.array([(dis_x+border*2)/2., (box_dis_y+border)/2., border/2.])
    boxes.append([halfEdge, center, quat])

    # front wall
    boxes.append([halfEdge, center, quat])
    
    # for test###################
    # floor
    halfEdge = np.array([dis_x/8., border/2., dis_z/2.])
    boxes.append([halfEdge, center, quat])

    # left wall
    halfEdge = np.array([border/2., (box_dis_y+border)/2 + 2*box_dis_y, dis_z/2.])
    boxes.append([halfEdge, center, quat])

    # right wall
    boxes.append([halfEdge, center, quat])

    # back wall
    halfEdge = np.array([(dis_x/4.+border*2)/2., (box_dis_y+border)/2. + 2*box_dis_y, border/2.])
    boxes.append([halfEdge, center, quat])

    # front wall
    boxes.append([halfEdge, center, quat])
    ######################
    return boxes


def calc_shape_states(x_curr, x_last, box_dis,time):
    global floor_move

    dis_x, dis_z = box_dis
    quat = np.array([1., 0., 0., 0.])

    states = np.zeros((10, dim_shape_state))

    states[0, :3] = np.array([x_curr, border/2., 0.])
    states[0, 3:6] = np.array([x_last, border/2., 0.])

    states[1, :3] = np.array([x_curr-(dis_x+border)/2., (box_dis_y+border)/2., 0.])
    states[1, 3:6] = np.array([x_last-(dis_x+border)/2., (box_dis_y+border)/2., 0.])

    states[2, :3] = np.array([x_curr+(dis_x+border)/2., (box_dis_y+border)/2., 0.])
    states[2, 3:6] = np.array([x_last+(dis_x+border)/2., (box_dis_y+border)/2., 0.])

    states[3, :3] = np.array([x_curr, (box_dis_y+border)/2., -(dis_z+border)/2.])
    states[3, 3:6] = np.array([x_last, (box_dis_y+border)/2., -(dis_z+border)/2.])

    states[4, :3] = np.array([x_curr, (box_dis_y+border)/2., (dis_z+border)/2.])
    states[4, 3:6] = np.array([x_last, (box_dis_y+border)/2., (dis_z+border)/2.])

    # for test
    dis_xx = dis_x*(3/8)
    dis_yy = box_dis_y*2
    
    # states[5, :3] = np.array([x_curr + dis_x*(3/8), border/2. + dis_yy, 0.])
    # states[5, 3:6] = np.array([x_last+ dis_x*(3/8), border/2. + dis_yy, 0.])
    
    if (time >= 300):
        floor_move += 0.15
    states[5, :3] = np.array([floor_move + dis_x*(3/8), border/2. + box_dis_y, 0.])
    states[5, 3:6] = np.array([floor_move + dis_x*(3/8), border/2. + box_dis_y, 0.])

    states[6, :3] = np.array([x_curr-(dis_x+border)/2. + dis_x*(3/4), (box_dis_y+2*border)+ dis_yy, 0.])
    states[6, 3:6] = np.array([x_last-(dis_x+border)/2.+ dis_x*(3/4), (box_dis_y+2*border)+ dis_yy, 0.])

    states[7, :3] = np.array([x_curr+(dis_x+border)/2., (box_dis_y+2*border)+ dis_yy, 0.])
    states[7, 3:6] = np.array([x_last+(dis_x+border)/2., (box_dis_y+2*border)+ dis_yy, 0.])

    states[8, :3] = np.array([x_curr+dis_xx, (box_dis_y+2*border)+ dis_yy, -(dis_z+border)/2.])
    states[8, 3:6] = np.array([x_last+dis_xx, (box_dis_y+2*border)+ dis_yy, -(dis_z+border)/2.])

    states[9, :3] = np.array([x_curr+dis_xx, (box_dis_y+2*border)+ dis_yy, (dis_z+border)/2.])
    states[9, 3:6] = np.array([x_last+dis_xx, (box_dis_y+2*border)+ dis_yy, (dis_z+border)/2.])
    # ################

    states[:, 6:10] = quat
    states[:, 10:] = quat

    return states



pyflex.init()


def rand_float(lo, hi):
    return np.random.rand() * (hi - lo) + lo

def rand_int(lo, hi):
    return np.random.randint(lo, hi)

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
radius = 0.1

box_dis_x = 10
box_dis_z = 3

scale = 1/(radius*0.55)

sx_f = scale*box_dis_x
sy_f = scale*box_dis_y
sz_f = scale*box_dis_z

x_center = 0
px_f = x_center - (box_dis_x - border) / 2.
py_f = border
pz_f = -(box_dis_z - border) / 2.

# py_f = 0.055 / 2. + border + 0.01

# 이 예제에서 안 쓰지만 sence param 맞추기 위해 기존 값 사용.



py_r = 0.0

# 0.0이면 false, 아니면 true
pz_r =  0.1
sx_r = 0.1
chk_drawPoint  = 0.1
chk_drawRender = 0.1

# scene에 값 넣기
cohesion = 0.02
suface = 0.072
scene_params = np.array([
    px_f, py_f, pz_f, sx_f, sy_f, sz_f,
    radius, py_r, pz_r, sx_r, chk_drawPoint, chk_drawRender,
    cohesion, suface,
    box_dis_x, box_dis_z])

pyflex.set_scene(8, scene_params, 0)
#pyflex.set_scene(10,np.array([]),0)
boxes = calc_box_init(np.array([0., 0., 0.]),box_dis_x, box_dis_z)


for i in range(len(boxes)):
    halfEdge = boxes[i][0]
    center = boxes[i][1]
    quat = boxes[i][2]
    pyflex.add_box(halfEdge, center, quat)

n_particles = pyflex.get_n_particles()
n_shapes = pyflex.get_n_shapes()
print(n_particles)


positions = np.zeros((time_step, n_particles, dim_position))
velocities = np.zeros((time_step, n_particles, dim_velocity))
shape_states = np.zeros((time_step, n_shapes, dim_shape_state))

x_box = 0


for i in range(time_step):
    x_box_last = x_box
    shape_states_ = calc_shape_states(x_box, x_box_last, scene_params[-2:],i)
    pyflex.set_shape_states(shape_states_)
    pyflex.step()

pyflex.clean()
