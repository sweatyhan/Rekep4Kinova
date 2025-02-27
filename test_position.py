import numpy as np
import json

with open('./robot_state.json', 'r') as f:
    data = json.load(f)
    
# 关键点在相机坐标系中的位置
keypoint_camera = np.array([0.11488139257068092, -0.13278498621805976, 1.154])

# 齐次变换矩阵 world2robot_homo
world2robot_homo = np.array(data['misc']['world2robot_homo'])

# 将关键点位置转换为齐次坐标
keypoint_camera_homo = np.append(keypoint_camera, 1)

# 使用齐次变换矩阵将关键点位置从相机坐标系转换到机器人坐标系
keypoint_robot_homo = np.dot(world2robot_homo, keypoint_camera_homo)

# 提取转换后的关键点位置
keypoint_robot = keypoint_robot_homo[:3]

# 打印
print("Keypoint position in robot frame:", keypoint_robot)
