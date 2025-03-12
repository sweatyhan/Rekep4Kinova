import pybullet as p
import pybullet_data
import numpy as np
import os
import subprocess
from scipy.spatial.transform import Rotation as R
# from kinova import KinovaRobot as kinova

class LocalIKSolver:
    def __init__(self, xacro_path="/home/kinova/Rekep4Real/kortex_description/robots/gen3_robotiq_2f_85.xacro"):
        urdf_path = self.convert_xacro_to_urdf(xacro_path)
        
        self.physics_client = p.connect(p.GUI)  # 使用 GUI 模式
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        urdf_dir = os.path.dirname(urdf_path)
        p.setAdditionalSearchPath(urdf_dir)
        model_dir = os.path.join(urdf_dir, './meshes')
        p.setAdditionalSearchPath(model_dir)
        start_pos = [0, 0, 0]  # 设置机器人的初始位置
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        self.robot_id = p.loadURDF(urdf_path, start_pos, start_orientation)
        self.num_joints = p.getNumJoints(self.robot_id)
        self.end_effector_index = self.get_gripper_link_index()  # 获取夹爪的链接索引
        # self.robot = rtb.models.URDF.KinovaGen3()
        self.error = (np.array([0.578613758, 0.00417445553, 0.438761622])-np.array([0.5773296356201172, 0.012200852856040001, 0.4267641603946686]))

    def convert_xacro_to_urdf(self, xacro_path):
        urdf_path = xacro_path.replace('.xacro', '.urdf')
        subprocess.run(['xacro', xacro_path, '-o', urdf_path], check=True)
        return urdf_path
    
    def get_gripper_link_index(self):
        # 获取夹爪的链接索引
        for i in range(self.num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            print(joint_info)
            if b'tool_frame' in joint_info[12]:  # 假设夹爪链接名称包含'gripper'
                return i
        return self.num_joints - 1  # 如果没有找到夹爪链接，返回最后一个链接索引

    def solve_ik(self, target_pos, target_ori, if_fixed_angles=False):
        # turn to euler-axis angles
        if if_fixed_angles:
            # target_ori = np.array(target_ori)
            # target_ori = target_ori[::-1]
            target_ori = p.getQuaternionFromEuler(target_ori)

        # add the error
        target_pos = np.array(target_pos) + self.error

        if len(target_ori) == 3:
            target_ori = p.getQuaternionFromEuler(target_ori)

        joint_positions = p.calculateInverseKinematics(
            self.robot_id,
            endEffectorLinkIndex=self.end_effector_index,
            targetPosition=target_pos,
            targetOrientation=target_ori,
            jointDamping=[0.001]*self.num_joints
        )
        return joint_positions[:7]  # 只返回相关的关节位置

    def is_target_reachable(self, target_pos, target_ori, tolerance=0.01):
        joint_positions = self.solve_ik(target_pos, target_ori)
        if np.any(np.isnan(joint_positions)) or np.any(np.isinf(joint_positions)):
            return False

        self.set_joint_positions(joint_positions)
        p.stepSimulation()
        ee_pos = np.array(self.get_end_effector_position()) + self.error
        distance = np.linalg.norm(np.array(ee_pos) - np.array(target_pos))
        return distance <= tolerance


    def set_joint_positions(self, joint_positions):
        if len(joint_positions) != self.num_joints:
            raise ValueError(f"Expected {self.num_joints} joint positions, but got {len(joint_positions)}")
        for i in range(self.num_joints):
            p.resetJointState(self.robot_id, i, joint_positions[i])

    def get_end_effector_position(self):
        state = p.getLinkState(self.robot_id, self.end_effector_index)
        return state[4]

    def disconnect(self):
        p.disconnect(self.physics_client)

def transform_to_B_frame_homogeneous(point, rotation_matrix, translation_vector):
    # 构建齐次变换矩阵
    homogeneous_matrix = np.eye(4)
    homogeneous_matrix[:3, :3] = rotation_matrix
    homogeneous_matrix[:3, 3] = translation_vector

    # 将点转换为齐次坐标
    point_homogeneous = np.append(point, 1)

    # 进行变换
    transformed_point_homogeneous = np.dot(homogeneous_matrix, point_homogeneous)

    # 返回转换后的点（去掉齐次坐标）
    return transformed_point_homogeneous[:3]

# Example usage
if __name__ == '__main__':
    urdf_path = "/home/kinova/Rekep4Real/kortex_description/arms/gen3/urdf/GEN3_URDF_V12.urdf"
    ik_solver = LocalIKSolver(urdf_path)

    # 定义坐标系 B 在坐标系 A 中的描述
    origin_B_in_A = np.array([0, 0, 0])
    x_B_in_A = np.array([0, 0, 1])
    y_B_in_A = np.array([0, 1, 0])
    z_B_in_A = np.array([-1, 0, 0])
    rotation_matrix_B_in_A = np.array([x_B_in_A, y_B_in_A, z_B_in_A]).T
    # get inverse
    rotation_matrix_B_in_A = np.linalg.inv(rotation_matrix_B_in_A)

    # 目标位置和方向
    target_pos_A = np.array([0.578613758, 0.00417445553, 0.438761622])
    target_ori_A = np.deg2rad([89.1808319, 1.55599177, 90.6496887])
    # target_ori_A = p.getQuaternionFromEuler(target_ori_A[::-1])

    # 将目标位置从坐标系 A 转换到坐标系 B
    # target_pos_B = transform_to_B_frame_homogeneous(target_pos_A, rotation_matrix_B_in_A, origin_B_in_A)
    target_pos_B = np.array([0.578613758, 0.00417445553, 0.438761622])
    # target_pos_B = np.array([10,10,10])

    # # 将目标方向从坐标系 A 转换到坐标系 B
    # rotation_matrix_A = p.getMatrixFromQuaternion(target_ori_A)
    # rotation_matrix_A = np.array(rotation_matrix_A).reshape(3, 3)
    # rotation_matrix_B = np.dot(rotation_matrix_B_in_A, rotation_matrix_A)
    # target_ori_B = R.from_matrix(rotation_matrix_B).as_quat()
    target_ori_B = target_ori_A

    print("Target position in frame B:", target_pos_B)
    print("Target orientation in frame B:", target_ori_B)

    joint_positions = ik_solver.solve_ik(target_pos_B, target_ori_B, if_fixed_angles=True)

    # kinova = kinova()
    # kinova.move_to_joint_positions(np.array(joint_positions)*180.0/np.pi)

    joint_positions = np.array(joint_positions)
    joint_positions = np.append(joint_positions, 0.0)
    print("Computed joint positions:", joint_positions)

    # 确保 joint_positions 数组的长度与机器人关节的数量一致
    if len(joint_positions) >= ik_solver.num_joints:
        joint_positions = joint_positions[:ik_solver.num_joints]
    else:
        raise ValueError(f"Expected at least {ik_solver.num_joints} joint positions, but got {len(joint_positions)}")

    # 设置机器人关节位置
    ik_solver.set_joint_positions(joint_positions)

    # 设置相机控制
    p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=50, cameraPitch=-35, cameraTargetPosition=[0, 0, 0])

    ee_pos = ik_solver.get_end_effector_position()
    print("End effector position:", ee_pos)

    # show simulation
    while True:
        p.stepSimulation()


    ik_solver.disconnect()