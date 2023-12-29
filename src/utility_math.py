import math
import numpy as np
import csv
import os


def sign(x):
    if x > 0:
        return 1
    elif x < 0:
        return -1
    else:
        return 0
    
def fhan(x1, x2, u, r, h):
    d = r * h
    d0 = d* h
    y = x1 - u+h*x2
    a0 = math.sqrt(d*d + 8*r*abs(y))

    if abs(y) <= d0:
        a = x2 + y/h
    else:
        a = x2+0.5*(a0-d)*sign(y)

    if abs(a)<=d:
        return -r*a/d
    else:
        return -r*sign(a)
    

def rotation_matrix_to_quaternion(R):
    qw = np.sqrt(1 + np.trace(R)) / 2
    qx = (R[2, 1] - R[1, 2]) / (4 * qw)
    qy = (R[0, 2] - R[2, 0]) / (4 * qw)
    qz = (R[1, 0] - R[0, 1]) / (4 * qw)
    
    return np.array([qw, qx, qy, qz])

def quaternion_to_rotation_matrix(q):
    """
    将四元数转换为旋转矩阵

    参数：
    q: 四元数，表示为 [w, x, y, z]

    返回：
    R: 旋转矩阵，3x3的NumPy数组
    """
    w, x, y, z = q
    R = np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
    ])
    return R

def quaternion_to_left_matrix(q):
    w, x, y, z = q
    Q_th = 0.5* np.array([
        [-x, -y, -z],
        [w, -z, y],
        [z, w, -x],
        [-y, x, w]
    ])
    return Q_th

def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

    # s = w*w + x*x+y*y+z*z


    return np.array([w, x, y, z])

def quaternion_conjugate(q):
    w, x, y, z = q
    return np.array([w, -x, -y, -z])

def quaternion_normalize(q):
    w,x,y,z = q
    s = w*w + x*x+y*y+z*z
    return np.array([w/s, x/s, y/s, z/s])




def csv_save(path, vector):
    if not os.path.exists(path):
        os.makedirs(path)
    with open(path,'a', newline='') as file:
        writer =csv.writer(file)
        writer.writerow(vector)
    return True

def state2Msg(p,q):
    pose = Pose()
    pose.position.x = p[0]
    pose.position.y = p[1]
    pose.position.z = p[2]

    pose.orientation.w = q[0]
    pose.orientation.x = q[1]
    pose.orientation.y = q[2]
    pose.orientation.z = q[3]
    return pose

# def quaternion_to_rotation_matrix(q):

#     w, x, y, z = q
#     R = np.array([
#         [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
#         [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
#         [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
#     ])
#     return R

def Msg2state(msg):
    p = np.array([0.0]*3)
    q = np.array([0.0]*4)


    p[0] = msg.position.x
    p[1] = msg.position.y
    p[2] = msg.position.z

    q[0] = msg.orientation.w
    q[1] = msg.orientation.x
    q[2] = msg.orientation.y
    q[3] = msg.orientation.z
    return p,q


def from_msg2T44(geo_msg):
    p,q = Msg2state(geo_msg)
    T44 = from_pq2T44(p,q)
    return T44

def from_pq2T44(p,q):
    T44 = np.zeros((4,4))
    T44[3,3] = 1.0

    R = quaternion_to_rotation_matrix(q)
    T44[:3,:3] = R
    T44[0,3] = p[0]
    T44[1,3] = p[1]
    T44[2,3] = p[2]
    return T44
# def rotation_matrix_to_quaternion(R):
#     qw = np.sqrt(1 + np.trace(R)) / 2
#     qx = (R[2, 1] - R[1, 2]) / (4 * qw)
#     qy = (R[0, 2] - R[2, 0]) / (4 * qw)
#     qz = (R[1, 0] - R[0, 1]) / (4 * qw)
    
#     return np.array([qw, qx, qy, qz])

def from_pose2T44(pose):
    p = pose[:3]
    q = pose[3:]
    T44 = from_pq2T44(p,q)
    return T44

def from_T442pq(T44):
    R = T44[:3,:3]

    q = rotation_matrix_to_quaternion(R)
    p = np.array([0.0]*3)
    p[0] = T44[0,3]
    p[1] = T44[1,3]
    p[2] = T44[2,3]
    
    return p,q
def from_T442pose(T44):
    p,q = from_T442pq(T44)
    pose = np.array([0.0]*7)
    pose[:3] = p
    pose[3:] = q/np.linalg.norm(q)
    return pose