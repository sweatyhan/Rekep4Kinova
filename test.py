import pybullet as p
import pybullet_data
import numpy as np
import os
from scipy.spatial.transform import Rotation as R

class LocalIKSolver:
    def __init__(self, urdf_path):
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
        self.num_links = p.getNumBodies()  # 获取链接数量
        self.end_effector_index = self.num_joints - 1  # 假设末端执行器是最后一个链接

    def solve_ik(self, target_pos, target_ori):
        joint_positions = p.calculateInverseKinematics(
            self.robot_id,
            endEffectorLinkIndex=self.end_effector_index,
            targetPosition=target_pos,
            targetOrientation=target_ori
        )
        return joint_positions

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

# Example usage
if __name__ == '__main__':
    urdf_path = "/home/kinova/Rekep4Real/kortex_description/arms/gen3/urdf/GEN3_URDF_V12.urdf"
    ik_solver = LocalIKSolver(urdf_path)

    info = p.getJointInfo(ik_solver.robot_id, 7)
    print(info)
    joint_state = p.getJointState(ik_solver.robot_id, 7)
    print(joint_state)
    ee_pos = ik_solver.get_end_effector_position()
    print("End effector position:", ee_pos)
    # 目标位置和方向（相对于世界坐标系）
    target_pos = [0.578613758, 0.00417445553, 0.438761622]
    target_ori = np.deg2rad([89.1808319, 1.55599177, 90.6496887])
    target_ori = p.getQuaternionFromEuler(target_ori)

    # 使用逆运动学求解器计算关节位置
    joint_positions = ik_solver.solve_ik(target_pos, target_ori)
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

    while True:
        p.stepSimulation()

    ik_solver.disconnect()