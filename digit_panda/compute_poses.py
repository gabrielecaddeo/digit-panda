import copy
import numpy as np
import pyquaternion as pyq
import trimesh
from scipy.spatial.transform import Rotation

#0.10900000000000007
poses = np.loadtxt('/home/gabriele/Desktop/gabriele/Gabriele/digit-panda/09_09_2022_10_07_41/poses.txt')
print(poses.shape)

transform_1 = pyq.Quaternion(axis=[0, 0, -1], angle=np.pi/4).transformation_matrix
transform_1[2,3] = 0.10900000000000007
transform_2 = pyq.Quaternion(axis=[0, 1, 0], angle=np.pi/2).transformation_matrix
transform_3 = pyq.Quaternion(axis=[0, 0, 1], angle=np.pi).transformation_matrix
transform_4 = np.eye(4)
transform_4[0, 3] = -0.02
transform_4[2, 3] = -0.015

f = open('poses_sensors.txt', 'w')
f.close()
for i in range(poses.shape[0]):
    if i == 0 or i == 1 or i ==4:
        pose = poses[i, :]
        digit = trimesh.load_mesh('/home/gabriele/Desktop/digit.STL')
        pose_digit = np.zeros((4,4))
        pose_digit[:, 0] = pose[:4]
        pose_digit[:, 1] = pose[4:8]
        pose_digit[:, 2] = pose[8:12]
        pose_digit[:, 3] = pose[12:]

        final_transform = pose_digit @ transform_1 @ transform_2  @ transform_3 @ transform_4 
        digit.apply_transform(final_transform)
        digit.export("/home/gabriele/Desktop/gabriele/Gabriele/digit-panda/new_digit"+str(i)+".STL")
        quat = pyq.Quaternion(matrix=final_transform)
        angle = quat.angle
        axis = quat.axis
        f = open('poses_sensors.txt', 'a')
        f.write(str(final_transform[0,3]) + ' ' + str(final_transform[1,3]) + ' ' + str(final_transform[2,3]) + ' ' + str(axis[0])  + ' ' + str(axis[1])  + ' ' + str(axis[2])  + ' ' + str(angle) + '\n')
    """pose = poses[i, :]
    digit = trimesh.load_mesh('/home/gabriele/Desktop/digit.STL')
    pose_digit = np.zeros((4,4))
    pose_digit[:, 0] = pose[:4]
    pose_digit[:, 1] = pose[4:8]
    pose_digit[:, 2] = pose[8:12]
    pose_digit[:, 3] = pose[12:]

    final_transform = pose_digit @ transform_1 @ transform_2  @ transform_3 @ transform_4 
    digit.apply_transform(final_transform)
    digit.export("/home/gabriele/Desktop/gabriele/Gabriele/digit-panda/digit"+str(i)+".STL")"""
