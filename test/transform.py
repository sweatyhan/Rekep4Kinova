import numpy as np
import json

with open('robot_state.json') as f:
    data = json.load(f)
    A = np.array(data['misc']['cam2robot_homo'])

B = np.linalg.inv(A)  # 求逆矩阵

# 定义向量 a
a = np.array([0.13920276776525708,
            -0.27213514058613325,
            0.97, 1.0])  # 添加齐次坐标

# 进行变换
transformed_a = np.dot(A, a)
transformed_b = np.dot(B, transformed_a)
transformed_c = np.dot(B, transformed_b)
transformed_d = np.dot(B, transformed_c)

# 打印结果
print("Transformed vector a:", transformed_a[:3])  # 去掉齐次坐标部分
print("Transformed vector b:", transformed_b[:3])  # 去掉齐次坐标部分
print("Transformed vector c:", transformed_c[:3])  # 去掉齐次坐标部分
print("Transformed vector d:", transformed_d[:3])  # 去掉齐次坐标部分