import pybullet as p
import pybullet_data
import numpy as np
import os
from kinova import KinovaRobot as kinova

class LocalIKSolver:
    def __init__(self, urdf_path):
        self.physics_client = p.connect(p.GUI)  # 使用 GUI 模式
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        urdf_dir = os.path.dirname(urdf_path)
        p.setAdditionalSearchPath(urdf_dir)
        model_dir = os.path.join(urdf_dir, './meshes')
        p.setAdditionalSearchPath(model_dir)
        self.robot_id = p.loadURDF(urdf_path)
        self.num_joints = p.getNumJoints(self.robot_id)

    def solve_ik(self, target_pos, target_ori):
        joint_positions = p.calculateInverseKinematics(
            self.robot_id,
            endEffectorLinkIndex=self.num_joints - 1,
            targetPosition=target_pos,
            targetOrientation=target_ori
        )
        return joint_positions

    def set_joint_positions(self, joint_positions):
        if len(joint_positions) != self.num_joints:
            raise ValueError(f"Expected {self.num_joints} joint positions, but got {len(joint_positions)}")
        for i in range(self.num_joints):
            p.resetJointState(self.robot_id, i, joint_positions[i])

    def disconnect(self):
        p.disconnect(self.physics_client)

# Example usage
if __name__ == '__main__':
    urdf_path = "/home/kinova/Rekep4Real/kortex_description/arms/gen3/urdf/GEN3_URDF_V12.urdf"
    ik_solver = LocalIKSolver(urdf_path)

    kinova = kinova()
    
    # 目标位置和方向
    target_pos = [0.578613758, 0.00417445553, 0.438761622]
    target_ori = np.deg2rad([89.1808319, 1.55599177, 90.6496887])
    target_ori = p.getQuaternionFromEuler(target_ori)
    
    joint_positions = ik_solver.solve_ik(target_pos, target_ori)
    joint_positions = np.array(joint_positions)
    print("Computed joint positions:", joint_positions)

    # 确保 joint_positions 数组的长度与机器人关节的数量一致
    if len(joint_positions) >= ik_solver.num_joints:
        joint_positions = joint_positions[:ik_solver.num_joints]
    else:
        raise ValueError(f"Expected at least {ik_solver.num_joints} joint positions, but got {len(joint_positions)}")

    # 设置机器人关节位置
    ik_solver.set_joint_positions(joint_positions)

    # 保持仿真环境打开
    while True:
        p.stepSimulation()

    ik_solver.disconnect()