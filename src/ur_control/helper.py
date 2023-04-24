#!/usr/bin/env python
import numpy as np
import os
from enum import Enum

import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Vector3
from tf import transformations

path_root = os.path.abspath(os.path.join(os.path.abspath(os.path.dirname(__file__)), '../'))


class DET_STATE(Enum):
    Fail = 1
    Success = 2

class GRIPPER_STATE(Enum):
    Ready = 0
    Sucking = 1
    Sucked = 2
    Releasing = 3
    Dropped = 4
    Stop = 5


def save_file(str, filepath):
    with  open(filepath, "w") as file:
        file.write(str)

def xyz_to_mat44(pos):
    return transformations.translation_matrix((pos.x, pos.y, pos.z))

def xyzw_to_mat44(ori):
    return transformations.quaternion_matrix((ori.x, ori.y, ori.z, ori.w))

def xyzrpy2mat44(xyzrpy):
    return np.dot(transformations.translation_matrix(xyzrpy[:3]), transformations.euler_matrix(*xyzrpy[3:]))


def gen_pose(x,y,z,rx,ry,rz,rw):
    pose=geometry_msgs.msg.Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = rx
    pose.orientation.y = ry
    pose.orientation.z = rz
    pose.orientation.w = rw
    return pose

def mat44_to_xyzrpy(matrix):
    xyz = transformations.translation_from_matrix(matrix)
    rpy = transformations.euler_from_matrix(matrix)
    return [xyz[0],xyz[1],xyz[2],rpy[0]*(180./np.pi),rpy[1]*(180./np.pi),rpy[2]*(180./np.pi)]


def mat44_to_xyzquat(matrix):
    xyz = transformations.translation_from_matrix(matrix)
    quat = transformations.quaternion_from_matrix(matrix)
    return [xyz[0],xyz[1],xyz[2],quat[0],quat[1],quat[2],quat[3]]

def mat44_to_pose(matrix):
    return gen_pose(*mat44_to_xyzquat(matrix))


def quat2msg(quat):
    """
    :param quat: rotation quaternion expressed as a tuple (x,y,z,w)
    :return: geometry_msgs.msg.Quaternion
    """
    q = Quaternion()
    q.x = quat[0]
    q.y = quat[1]
    q.z = quat[2]
    q.w = quat[3]
    return q


def pose_to_mat44(pose):
    """
    :param pose: geometry_msgs.msg.Pose
    :return:
    """
    return np.dot(xyz_to_mat44(pose.position), xyzw_to_mat44(pose.orientation))


def mat44_to_transform(mat44):
    transform = geometry_msgs.msg.Transform()
    xyz = transformations.translation_from_matrix(mat44)
    transform.translation = Vector3(*xyz)
    transform.rotation = quat2msg(transformations.quaternion_from_matrix(mat44))
    return transform

def transrot_to_mat44(translation, rotation):
    """
    :param translation: translation expressed as a tuple (x,y,z)
    :param rotation: rotation quaternion expressed as a tuple (x,y,z,w)
    :return: a :class:`numpy.matrix` 4x4 representation of the transform
    :raises: any of the exceptions that :meth:`~tf.Transformer.lookupTransform` can raise

    Converts a transformation from :class:`tf.Transformer` into a representation as a 4x4 matrix.
    """

    return np.dot(transformations.translation_matrix(translation), transformations.quaternion_matrix(rotation))


def transform_to_mat44(transform):
    t = transform.translation
    r = transform.rotation
    return transrot_to_mat44([t.x, t.y, t.z], [r.x, r.y, r.z, r.w])


def transform_point_from_tf(mat44, point):
    xyz = tuple(np.dot(mat44, np.array([point.x, point.y, point.z, 1.0])))[:3]
    point = geometry_msgs.msg.Point(*xyz)
    return point

# def quaternion_rotation_matrix(Q):
#     """
#     Covert a quaternion into a full three-dimensional rotation matrix.
 
#     Input
#     :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
#     Output
#     :return: A 3x3 element matrix representing the full 3D rotation matrix. 
#              This rotation matrix converts a point in the local reference 
#              frame to a point in the global reference frame.
#     """
#     # Extract the values from Q
#     q0 = Q[0]
#     q1 = Q[1]
#     q2 = Q[2]
#     q3 = Q[3]
     
#     # First row of the rotation matrix
#     r00 = 2 * (q0 * q0 + q1 * q1) - 1
#     r01 = 2 * (q1 * q2 - q0 * q3)
#     r02 = 2 * (q1 * q3 + q0 * q2)
     
#     # Second row of the rotation matrix
#     r10 = 2 * (q1 * q2 + q0 * q3)
#     r11 = 2 * (q0 * q0 + q2 * q2) - 1
#     r12 = 2 * (q2 * q3 - q0 * q1)
     
#     # Third row of the rotation matrix
#     r20 = 2 * (q1 * q3 - q0 * q2)
#     r21 = 2 * (q2 * q3 + q0 * q1)
#     r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
#     # 3x3 rotation matrix
#     rot_matrix = np.array([[r00, r01, r02],
#                            [r10, r11, r12],
#                            [r20, r21, r22]])
                            
#     return rot_matrix