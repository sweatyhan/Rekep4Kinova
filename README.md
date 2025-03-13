# ReKep Project

The ReKep project aims to transition ReKep from Isaac Sim to using the Kinova Gen3 robotic arm. This project builds upon the outstanding work of [huangwl18/ReKep](https://github.com/huangwl18/ReKep) and [Everloom-129/ReKep](https://github.com/Everloom-129/ReKep).

## Table of Contents

- [Introduction](#introduction)
- [Installation](#installation)
- [Usage](#usage)
- [Running the Project](#running-the-project)
- [Contributing](#contributing)
- [License](#license)

## Introduction

The ReKep project leverages the Kinova Gen3 robotic arm to perform various tasks. The project is designed to be modular and extensible, allowing for easy integration with other systems and components.

## Installation

To get started with the ReKep project, follow these steps:

1. Create and activate vir-environment 
    python3.10 is recommended, since my python version is this(


2. Clone the repository:
    ```bash
    git clone https://github.com/sweatyhan/ReKep4Kinova.git
    cd ReKep4Kinova
    ```

3. Install Kortex-api for Kinova Gen3:
    Follow the instruction in https://github.com/Kinovarobotics/Kinova-kortex2_Gen3_G3L/blob/master/readme.md
    Choose kortex_api-2.6.0.post3-py3-none-any.whl for Kinova Gen3

4. Install the required dependencies:
    ```bash
    pip install -r requirements.txt
    ```
    Note: 
    If you are told that torch==2.5.1+cu121 is not found, you can try this command below and do step 4 again.
    ```bash
    pip install torch==2.5.1+cu121 --extra-index-url https://download.pytorch.org/whl/cu121  # -i https://pypi.tuna.tsinghua.edu.cn/simple
    ```
    After the last step, you might encounter an error: *kortex-api 2.6.0.post3 requires protobuf==3.5.1, but you have protobuf 3.20.1 which is incompatible.* This error can be safely ignored as kortex-api 2.6.0 works well in practice despite the version mismatch.
    

## Usage

The project consists of several scripts that work together to control the Kinova Gen3 robotic arm. Below is a brief description of each script:

- [photo.py](http://_vscodecontentref_/0): Captures images using the camera.
- [main_vision.py](http://_vscodecontentref_/1): Processes the captured images and performs vision-based tasks.
- [r2d2_rekep.py](http://_vscodecontentref_/2): Controls the Kinova Gen3 robotic arm based on the processed vision data.

## Running the Project

To run the project, follow these steps in order:

1. Run the [photo.py](http://_vscodecontentref_/3) script to capture images:
    ```bash
    python photo.py --data_path --frame_number
    ```

2. Run the [main_vision.py](http://_vscodecontentref_/4) script to process the captured images:
    ```bash
    python main_vision.py --
    ```

3. Finally, run the [r2d2_rekep.py](http://_vscodecontentref_/5) script to control the Kinova Gen3 robotic arm:
    ```bash
    python r2d2_rekep.py --
    ```

## State

The project is nearly complete. The IK solver is implemented using the PyBullet simulator, and the model file used is `gen3_robotiq_2f_85.xacro`, located in the `kortex_description/robots` directory.

## Addition

If the robot you are using is not Kinova Gen3 and robotiq-2df-85, `robotics-toolbox-python` is suggested to use. 
Besides PyBullet, I also tried using `robotics-toolbox-python`. This package has a rich library of robotic arm models, and its IK solver is a bit more convenient. However, I encountered errors when trying to solve the IK problem for the Kinova Gen3, while it worked well with other robot models. Therefore, I reverted to using PyBullet.

## Contributing

We welcome contributions to the ReKep project! If you have any ideas, bug reports, or pull requests, please feel free to submit them. Follow these steps to contribute:

1. Fork the repository.
2. Create a new branch for your feature or bugfix.
3. Commit your changes and push them to your fork.
4. Submit a pull request with a detailed description of your changes.
