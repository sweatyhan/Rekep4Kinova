"""
Adapted from OmniGibson and the Lula IK solver
"""
# import omnigibson.lazy as lazy

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2
from kortex_api.Exceptions.KServerException import KServerException
from scipy.spatial.transform import Rotation as R

class JointAngle:
    def __init__(self, value):
        self.value = value

class ComputedJointAngles:
    def __init__(self, joint_angles):
        self.joint_angles = [JointAngle(angle) for angle in joint_angles]

class IKResult:
    def __init__(self, success, joint_positions, error_pos, error_rot, num_descents):
        self.success = success
        self.joint_positions = joint_positions
        self.error_pos = error_pos
        self.error_rot = error_rot
        self.num_descents = num_descents

import numpy as np
from scipy.spatial.transform import Rotation

class IKResult:
    """Class to store IK solution results"""
    def __init__(self, success, joint_positions, error_pos, error_rot, num_descents=None):
        self.success = success
        self.cspace_position = joint_positions
        self.position_error = error_pos
        self.rotation_error = error_rot
        self.num_descents = num_descents if num_descents is not None else 1
    
    
# TODO use real IK solver
class FrankaIKSolver:
    """Franka IK Solver"""
    def __init__(self, reset_joint_pos, world2robot_homo=None):
        # DH parameters for Franka (simplified version)
        self.dh_params = {
            'd1': 0.333,   # Joint 1
            'd3': 0.316,   # Joint 3
            'd5': 0.384,   # Joint 5
            'd7': 0.107,   # End effector
            'a7': 0.088,   # End effector
        }
        
        # Joint limits (in radians)
        self.joint_limits = {
            'lower': np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]),
            'upper': np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
        }
        
        # Default home position
        self.reset_joint_pos = reset_joint_pos
        
        # Transform from world to robot base
        self.world2robot_homo = world2robot_homo if world2robot_homo is not None else np.eye(4)

    def transform_pose(self, pose_homo):
        """Transform pose from world frame to robot base frame"""
        return np.dot(self.world2robot_homo, pose_homo)
    
    def solve(self, target_pose_homo, 
             position_tolerance=0.01,
             orientation_tolerance=0.05,
             max_iterations=150,
             initial_joint_pos=None):
        """
        Mock IK solver that returns a valid IKResult
        """
        # Transform target pose to robot base frame
        robot_pose = self.transform_pose(target_pose_homo)
        
        # Extract position and rotation
        target_pos = robot_pose[:3, 3]
        target_rot = robot_pose[:3, :3]
        
        # Use initial joint positions or default
        if initial_joint_pos is None:
            initial_joint_pos = self.reset_joint_pos
        
        # 简单的工作空间检查
        in_workspace = np.all(np.abs(target_pos) < 1.0)
        
        if 1: #in_workspace:
            # 成功情况
            return IKResult(
                success=True,
                joint_positions=initial_joint_pos,  # 使用初始关节角度或默认值
                error_pos=0.01,
                error_rot=0.01,
                num_descents=max_iterations // 2
            )
        else:
            # 失败情况，但仍然返回一个有效的IKResult
            return IKResult(
                success=False,
                joint_positions=self.reset_joint_pos,  # 使用重置位置
                error_pos=1.0,
                error_rot=1.0,
                num_descents=max_iterations
            )
    
    def forward_kinematics(self, joint_positions):
        """
        Compute forward kinematics (placeholder)
        
        Args:
            joint_positions (array): Joint angles
            
        Returns:
            4x4 array: Homogeneous transformation matrix
        """
        # Placeholder - implement actual FK
        return np.eye(4)

# TODO use real IK solver

class KinovaIKSolver:
    """Kinova IK Solver"""
    def __init__(self, kinova, reset_joint_pos, world2robot_homo=None):
        # self.ip_address = ip_address
        # self.username = username
        # self.password = password
        # self.port = port
        # self.port_real_time = port_real_time
        self.kinova = kinova
        self.reset_joint_pos = reset_joint_pos
        self.world2robot_homo = world2robot_homo if world2robot_homo is not None else np.eye(4)

    def transform_pose(self, target_pose_homo):
        # Transform target pose to robot base frame
        return np.dot(self.world2robot_homo, target_pose_homo)

    def solve(self, target_pose_homo, 
              position_tolerance=0.01,
              orientation_tolerance=0.05,
              max_iterations=150,
              initial_joint_pos=None):
        """
        Kinova IK solver that returns a valid IKResult
        """
        # Transform target pose to robot base frame
        robot_pose = self.transform_pose(target_pose_homo)
        # robot_pose = target_pose_homo
        
        # Extract position and rotation
        target_pos = robot_pose[:3, 3]
        target_rot = robot_pose[:3, :3]

        # change totation matrix into euler angle
        r = R.from_matrix(target_rot)
        euler_angles = r.as_euler('xyz', degrees=True)
        
        # Use initial joint positions or default
        if initial_joint_pos is None:
            initial_joint_pos = self.reset_joint_pos
        
        # Create IKData object
        input_IkData = Base_pb2.IKData()
        input_IkData.cartesian_pose.x = target_pos[0]
        input_IkData.cartesian_pose.y = target_pos[1]
        input_IkData.cartesian_pose.z = target_pos[2]
        input_IkData.cartesian_pose.theta_x = euler_angles[0]
        input_IkData.cartesian_pose.theta_y = euler_angles[1]
        input_IkData.cartesian_pose.theta_z = euler_angles[2]

        # Fill the IKData Object with the guessed joint angles
        for joint_angle in initial_joint_pos:
            jAngle = input_IkData.guess.joint_angles.add()
            jAngle.value = joint_angle
        
        try:
            print("Computing Inverse Kinematics using joint angles and pose...")
            computed_joint_angles = self.kinova.CptIK(input_IkData)
            success = True
            for joint_angle in computed_joint_angles.joint_angles:
                joint_angle.value = joint_angle.value/180*np.pi
            error_pos = 0.01  # Placeholder for actual position error
            error_rot = 0.01  # Placeholder for actual rotation error
            num_descents = max_iterations // 2  # Placeholder for actual number of descents
        except KServerException as ex:
            print("Unable to compute inverse kinematics")
            print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
            print("Caught expected error: {}".format(ex))
            success = False
            computed_joint_angles = self.reset_joint_pos
            computed_joint_angles = ComputedJointAngles(joint_angles=computed_joint_angles)
            error_pos = 1.0
            error_rot = 1.0
            num_descents = max_iterations
        
        joint_positions = []
        for joint_angle in computed_joint_angles.joint_angles:
            joint_positions.append(joint_angle.value)
        
        print(joint_positions)
        
        return IKResult(
            success=success,
            joint_positions=joint_positions,
            error_pos=error_pos,
            error_rot=error_rot,
            num_descents=num_descents
        )

# Unit tests
def test_franka_ik():
    # Create solver
    solver = FrankaIKSolver()
    
    # Test case 1: Identity pose
    target = np.eye(4)
    result = solver.solve(target)
    assert result['success']
    
    # Test case 2: Transformed pose
    target = np.array([
        [1, 0, 0, 0.5],
        [0, 1, 0, 0.0],
        [0, 0, 1, 0.3],
        [0, 0, 0, 1.0]
    ])
    result = solver.solve(target)
    assert result['success']
    
    # Test case 3: Check joint limits
    joints = result['joint_positions']
    assert np.all(joints >= solver.joint_limits['lower'])
    assert np.all(joints <= solver.joint_limits['upper'])
    
    print("All tests passed!")

class IKSolver:
    """
    Class for thinly wrapping Lula IK solver
    
    This class implements inverse kinematics (IK) for robotic manipulators.
    IK is the process of calculating joint angles needed to achieve a desired
    end-effector pose. This is essential for robot motion planning and control.
    
    The solver uses Cyclic Coordinate Descent (CCD), an iterative method that:
    1. Optimizes one joint at a time
    2. Minimizes position and orientation error of end-effector
    3. Respects joint limits and collision constraints
    4. Handles redundant manipulators (robots with >6 DOF)
    """

    def __init__(
        self,
        robot_description_path,
        robot_urdf_path,
        eef_name,
        reset_joint_pos,
        world2robot_homo,
    ):
        # Create robot description, kinematics, and config
        # self.robot_description = lazy.lula.load_robot(robot_description_path, robot_urdf_path)
        # self.kinematics = self.robot_description.kinematics()
        # self.config = lazy.lula.CyclicCoordDescentIkConfig()
        self.eef_name = eef_name
        self.reset_joint_pos = reset_joint_pos
        self.world2robot_homo = world2robot_homo

    def solve(
        self,
        target_pose_homo,
        position_tolerance=0.01,
        orientation_tolerance=0.05,
        position_weight=1.0,
        orientation_weight=0.05,
        max_iterations=150,
        initial_joint_pos=None,
    ):
        """
        Backs out joint positions to achieve desired @target_pos and @target_quat

        The solver uses an optimization approach to find joint angles that place the
        end-effector at the target pose. It balances:
        - Position accuracy (xyz coordinates)
        - Orientation accuracy (rotation matrix)
        - Joint limits
        - Solution convergence speed

        Args:
            target_pose_homo (np.ndarray): [4, 4] homogeneous transformation matrix of the target pose in world frame
            position_tolerance (float): Maximum position error (L2-norm) for a successful IK solution
            orientation_tolerance (float): Maximum orientation error (per-axis L2-norm) for a successful IK solution
            position_weight (float): Weight for the relative importance of position error during CCD
            orientation_weight (float): Weight for the relative importance of position error during CCD
            max_iterations (int): Number of iterations used for each cyclic coordinate descent.
            initial_joint_pos (None or n-array): If specified, will set the initial cspace seed when solving for joint
                positions. Otherwise, will use self.reset_joint_pos

        Returns:
            ik_results (lazy.lula.CyclicCoordDescentIkResult): IK result object containing the joint positions and other information.
        """
        # convert target pose to robot base frame
        # target_pose_robot = np.dot(self.world2robot_homo, target_pose_homo)
        # target_pose_pos = target_pose_robot[:3, 3]
        # target_pose_rot = target_pose_robot[:3, :3]
        # ik_target_pose = lazy.lula.Pose3(lazy.lula.Rotation3(target_pose_rot), target_pose_pos)
        # Set the cspace seed and tolerance
        initial_joint_pos = self.reset_joint_pos if initial_joint_pos is None else np.array(initial_joint_pos)
        # self.config.cspace_seeds = [initial_joint_pos]
        # self.config.position_tolerance = position_tolerance
        # self.config.orientation_tolerance = orientation_tolerance
        # self.config.ccd_position_weight = position_weight
        # self.config.ccd_orientation_weight = orientation_weight
        # self.config.max_num_descents = max_iterations
        # Compute target joint positions
        return None
        # ik_results = lazy.lula.compute_ik_ccd(self.kinematics, ik_target_pose, self.eef_name, self.config)
        # return ik_results


if __name__ == "__main__":
    test_franka_ik()
