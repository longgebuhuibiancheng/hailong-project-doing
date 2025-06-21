import numpy as np
from scipy.spatial.transform import Rotation as R

def convert(x, y, z, x1, y1, z1, rx, ry, rz):

    """    
    使用深度相机识别到的物体坐标（x, y, z）和机械臂末端的位姿（x1,y1,z1,rx,ry,rz
    ）来计算物体在机械臂基坐标下的位置（x, y, z）
    

    Args:
        x : 相机坐标系下物体位置x
        y : 相机坐标系下物体位置y
        z : 相机坐标系下物体位置z
        x1 : 机械臂末端位姿 x
        y1 : 机械臂末端位姿 y
        z1 : 机械臂末端位姿 z
        rx : 机械臂末端位姿 rx
        ry : 机械臂末端位姿 ry
        rz : 机械臂末端位姿 rz

    Returns:
        _type_: 物体在机械臂基坐标系下的位置 x y z
    """

    # 相机坐标系到机械臂末端坐标系的旋转矩阵和平移向量
    #rotation_matrix = np.array([[0.16603618, -0.98582585, 0.02407048],[ 0.9852236, 0.16479554, -0.04665719],[ 0.04202916, 0.03146159, 0.99862091]])
    rotation_matrix = np.array([[ 0.00760636, -0.99889634,  0.04634921],
 [ 0.99924795,  0.00583029, -0.03833465],
 [ 0.03802211,  0.04660594,  0.99818946]]
)

    #translation_vector = np.array([-0.066, 0.03, 0.016])
    #translation_vector = np.array([0.04698602559897092, -0.030067631489077935, -0.026264674585472995])
    translation_vector = np.array([0.077,-0.033, -0.16])
    obj_camera_coordinates = np.array([x, y, z])     # 深度相机识别物体返回的坐标

    end_effector_pose = np.array([x1, y1, z1,
                                  rx, ry, rz])     # 机械臂末端的位姿，单位为弧度

    # 将旋转矩阵和平移向量转换为齐次变换矩阵
    T_camera_to_end_effector = np.eye(4)
    T_camera_to_end_effector[:3, :3] = rotation_matrix
    T_camera_to_end_effector[:3, 3] = translation_vector
    # 机械臂末端的位姿转换为齐次变换矩阵
    position = end_effector_pose[:3]
    orientation = R.from_euler('xyz', end_effector_pose[3:], degrees=False).as_matrix()
    T_base_to_end_effector = np.eye(4)
    T_base_to_end_effector[:3, :3] = orientation
    T_base_to_end_effector[:3, 3] = position
    # 计算物体相对于机械臂基座的位姿
    obj_camera_coordinates_homo = np.append(obj_camera_coordinates, [1])  # 将物体坐标转换为齐次坐标
    # obj_end_effector_coordinates_homo = np.linalg.inv(T_camera_to_end_effector).dot(obj_camera_coordinates_homo)
    obj_end_effector_coordinates_homo = T_camera_to_end_effector.dot(obj_camera_coordinates_homo)
    obj_base_coordinates_homo = T_base_to_end_effector.dot(obj_end_effector_coordinates_homo)
    obj_base_coordinates = obj_base_coordinates_homo[:3]  # 从齐次坐标中提取物体的x, y, z坐标

    x, y, z = obj_base_coordinates

    return x, y, z