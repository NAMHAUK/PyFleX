import os
import numpy as np
import pyflex
import time
import torch


des_dir = 'test_RigidFall'
os.system('mkdir -p ' + des_dir)


def rand_float(lo, hi):
    return np.random.rand() * (hi - lo) + lo


pyflex.init()
n_instance = 2

scene_params = np.zeros(n_instance * 3 + 3)
scene_params[0] = n_instance

low_bound = 0.09
# for i in range(n_instance):
#     x = rand_float(0., 0.1)
#     y = rand_float(low_bound, low_bound + 0.01)
#     z = rand_float(0., 0.1)

#     scene_params[i * 3 + 1] = x
#     scene_params[i * 3 + 2] = y
#     scene_params[i * 3 + 3] = z

#     low_bound += 1.71

scene_params[1] = 1.0
scene_params[2] = 0.5
scene_params[3] = 0.

scene_params[4] = 0.
scene_params[5] = 0.5
scene_params[6] = 0.

# rigidStiffness, springStiffness
scene_params[7] = 0.5
scene_params[8] = 0.0


# scene_params = np.array([
#     px_f, py_f, pz_f, sx_f, sy_f, sz_f,
#     px_r, py_r, pz_r, sx_r, sy_r, sz_r,
#     box_dis_x, box_dis_z])

pyflex.set_scene(3, scene_params, 0)
# print("Scene Upper:", pyflex.get_scene_lower())
# print("Scene Lower:", pyflex.get_scene_upper())

leng = 1.0
for i in range(330):
    pyflex.step()
    pyflex.get_n_rigids(leng, 0)
    if(leng > 0.0):
        leng -= 0.01
pyflex.clean()
