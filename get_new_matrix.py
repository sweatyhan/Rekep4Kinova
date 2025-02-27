import numpy as np

# 新的平移向量
translation = np.array([1.75, 0.1, 0.37])

# 新的旋转矩阵
# x方向 (-1, 0, 0)
# y方向 (0, 1/2, -sqrt(3)/2)
# z方向是x和y的叉积
# x_direction = np.array([-1, 0, 0])
# y_direction = np.array([0, 1/2, -np.sqrt(3)/2])
# z_direction = np.cross(x_direction, y_direction)
x_direction = np.array([0, -1, 0])
z_direction = np.array([-np.sqrt(3)/2, 0, -1/2])
y_direction = np.cross(x_direction, z_direction)

rotation_matrix = np.vstack([x_direction, y_direction, z_direction]).T

# 组合平移向量和旋转矩阵形成新的齐次变换矩阵
transformation_matrix = np.eye(4)
transformation_matrix[:3, :3] = rotation_matrix
transformation_matrix[:3, 3] = translation

print("新的齐次变换矩阵：")
print(transformation_matrix)