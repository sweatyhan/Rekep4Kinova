## Introduction

This document highlights the key aspects of the code in `r2d2_rekep.py` that you should focus on.

The `execute()` function is the core of `r2d2_rekep.py`. It takes constraints and keypoints from the previous step, generates subgoals and subpaths, and ultimately drives the robot to execute the paths.

During this process, keypoints are transformed from the camera's frame to the robot's frame. The robot arm's API is utilized throughout, including optimizing subgoals and subpaths, retrieving the robot's states, and controlling the robot. These aspects will be discussed in detail below.

## Transformation of Keypoints

The first step is to verify the frame of your robot arm. Different robot arms may have varying frame definitions. For example, in the Kinova Gen3, the x-axis points forward, the y-axis points to the left, and the z-axis points upward, with the origin located at the robot's base. You can find this information in the technical documentation of your robot arm.

### Modify the Prompt

After verifying the frame, check the file [vlm_query/prompt_template.txt]. This file contains a sentence describing the robot's frame to the VLM, which you may need to adjust according to your robot's frame.

### Modify the Matrix

The transformation matrix is stored in [robot_state.json]['misc']['cam2robot_homo'].

Before calling `execute()`, the `transform_keypoints()` function is executed to convert the keypoints' frame. Ensure the matrix is accurate to allow the robot to move to the correct positions.

The script `get_transform_matrix.py` provides an example of how to compute the matrix using the camera's position and orientation in the robot's frame.

## Robot API

All interactions between the program and the robot are handled through the `KinovaIKSolver` class. If you are not using the Kinova Gen3, ensure you verify the format of the state values returned by your robot. The optimization of subgoals and subpaths requires angles in radians, positions in meters, and orientations in quaternions. Additionally, check whether the end effector's orientation is defined in a moving-axis (Euler-axis) or fixed-axis system.

### IK Solver

The IK solver is a critical function for optimization. It takes a guessed position and outputs the joint positions.

The Kinova Gen3 has a built-in IK solver in its controller, accessible via the robot's API. However, connecting to the robot for every IK computation is time-consuming, especially since optimization requires thousands of IK calculations. Therefore, it is recommended to use a local IK solver for faster optimization.

Ideally, the manufacturer of your robot arm should provide a local library for IK solving. However, since I couldn't find one for my robot arm, I had to develop a custom solver.

In this project, PyBullet is used as the IK solver. If you are not using the Kinova Gen3, consider using the `robotic-toolbox-python` library. This library not only provides joint positions but also indicates whether the target position is reachable.

When loading the robot model, ensure you use a model that includes the end effector, not just the robot arm. Differences in the definition of the end effector between the real robot and the model can lead to discrepancies in the position and orientation of the end effector during IK calculations.

If you are using pybullet, remember to make the jointDamping less then 0.01 just like `jointDamping=[0.001]*self.num_joints` when calling `p.calculateInverseKinematics()`