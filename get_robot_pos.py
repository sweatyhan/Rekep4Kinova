import numpy as np
from scipy.spatial.transform import Rotation as R
from kinova import KinovaRobot as kinova

kinova = kinova()
def kinova_get_joint_pos():
    joint_pos = kinova.get_joint_positions()
    joint_pos = np.array(joint_pos)
    joint_pos = joint_pos /180.0*np.pi
    return joint_pos

def kinova_get_ee_pos():
    ee_pos = kinova.get_tool_position()
    ee_pos = np.array(ee_pos)
    # position = ee_pos[:3]
    euler_angles = ee_pos[3:]

    # 将欧拉角从度转换为弧度
    euler_angles_rad = np.deg2rad(euler_angles)

    # # 将欧拉角转换为四元数
    # rotation = R.from_euler('xyz', euler_angles_rad)
    # quat = rotation.as_quat()
    # ee_pos = np.concatenate((position, quat))

    return ee_pos, euler_angles_rad

joint_pos = kinova_get_joint_pos()
print(joint_pos)
ee_pos, euler_angles_rad = kinova_get_ee_pos()
print(ee_pos)
print(euler_angles_rad/np.pi*180)