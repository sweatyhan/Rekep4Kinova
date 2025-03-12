import pybullet as p
import pybullet_data
import numpy as np
import os
import subprocess
from scipy.spatial.transform import Rotation as R

from spatialmath import SE3
import spatialgeometry as sg
import matplotlib.pyplot as plt
import roboticstoolbox as rtb
from kinova import KinovaRobot as kinova

class LocalIKSolver:
    def __init__(self, xacro_path):
        # Convert Xacro to URDF
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

    def solve_ik_p(self, target_pos, target_ori, if_fixed_base=False):
        # turn to euler-axis angles
        if if_fixed_base:
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
        return joint_positions[:self.num_joints]  # 只返回相关的关节位置
    
    def solve_ik_r(self, target_pos, target_ori):
        # Create the target transformation matrix
        target_transform = SE3(target_pos) * SE3.RPY(target_ori, unit='rad')
        
        q0 = self.robot.qr
        print(q0)
        # Solve inverse kinematics
        ik_solution = self.robot.ikine_LM(target_transform, q0=q0)
        if ik_solution.success:
            return ik_solution.q
        else:
            raise ValueError("IK solution not found")

    def set_joint_positions(self, joint_positions):
        # if len(joint_positions) != self.num_joints:
        #     raise ValueError(f"Expected {self.num_joints} joint positions, but got {len(joint_positions)}")
        for i in range(len(joint_positions)):
            p.resetJointState(self.robot_id, i, joint_positions[i])

    def get_end_effector_state(self):
        state = p.getLinkState(self.robot_id, self.end_effector_index)
        pos = state[4]
        ori = state[5]
        return pos, ori

    def disconnect(self):
        p.disconnect(self.physics_client)

# Example usage
if __name__ == '__main__':
    xacro_path = "/home/kinova/Rekep4Real/kortex_description/robots/gen3_robotiq_2f_85.xacro"
    ik_solver = LocalIKSolver(xacro_path)
    real_robot = kinova(if_pybullet=False)

    # 目标位置和方向（相对于世界坐标系）
    target_pos = [0.578613758, 0.00417445553, 0.438761622]
    target_ori = np.deg2rad([89.17977142, 1.55649638, 90.65080261])
    target_ori_r = target_ori
    target_ori_p = target_ori

    # 使用逆运动学求解器计算关节位置
    joint_positions_p = ik_solver.solve_ik_p(target_pos, target_ori_p, if_fixed_base=True)

    print("Computed joint positions using PyBullet IK:", joint_positions_p)

    # 设置机器人关节位置并获取末端执行器状态
    # joint_positions_angles = np.array([0,0,0,0,30,0,0])
    # joint_positions_p = np.append(joint_positions_p, 0)
    # joint_positions_p = np.array(joint_positions_p)/180*np.pi
    joint_positions_angles = np.array(joint_positions_p)*180/np.pi

    ik_solver.set_joint_positions(joint_positions_p)
    real_robot.move_to_joint_positions(joint_positions_angles[:7])

    ee_pos_p, ee_ori_p = ik_solver.get_end_effector_state()
    ee_ori_p = p.getEulerFromQuaternion(ee_ori_p)
    ee_ori_p = np.rad2deg(ee_ori_p)
    ori_pos_real = real_robot.get_tool_position()
    ee_pos_real = ori_pos_real[:3]
    ee_ori_real = ori_pos_real[3:]

    print("End effector position using PyBullet IK:", ee_pos_p)
    print("End effector orientation using PyBullet IK:", ee_ori_p)
    print("End effector position using real robot:", ee_pos_real)
    print("End effector orientation using real robot:", ee_ori_real)

    # 设置相机控制
    p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=50, cameraPitch=-35, cameraTargetPosition=[0, 0, 0])

    while True:
        p.stepSimulation()

    ik_solver.disconnect()