import sys
import os
import threading
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2
from kortex_api.Exceptions.KServerException import KServerException
import pybullet as p
import pybullet_data

TIMEOUT_DURATION = 100

class LocalIKSolver:
    def __init__(self, urdf_path="/home/kinova/Rekep4Real/kortex_description/arms/gen3/urdf/GEN3_URDF_V12.urdf"):
        self.physics_client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
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

    def disconnect(self):
        p.disconnect(self.physics_client)


class KinovaRobot:
    def __init__(self, ip_address="192.168.1.10", username="admin", password="admin", port=10000, port_real_time=10001):
        self.ip_address = ip_address
        self.username = username
        self.password = password
        self.port = port
        self.port_real_time = port_real_time
        self.ik_solver = LocalIKSolver()

        sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
        import Kinova_api.api_python.examples.utilities as utilities
        self.utilities = utilities

        self.args = self.utilities.parseConnectionArguments()
        # # self.router = self.utilities.DeviceConnection.createTcpConnection(args)
        # # self.router_real_time = self.utilities.DeviceConnection.createUdpConnection(args)
        # with self.utilities.DeviceConnection.createTcpConnection(args) as router:
        #     self.router = router
        # with self.utilities.DeviceConnection.createUdpConnection(args) as router_real_time:
        #     self.router_real_time = router_real_time
        # self.base = BaseClient(self.router_real_time)
        # self.base_cyclic = BaseCyclicClient(self.router_real_time)

    def check_for_end_or_abort(self, e):
        def check(notification):
            if notification.action_event == Base_pb2.ACTION_END or notification.action_event == Base_pb2.ACTION_ABORT:
                e.set()
        return check

    def get_tool_position(self):
        with self.utilities.DeviceConnection.createTcpConnection(self.args) as router:
            base_cyclic = BaseCyclicClient(router)

            try:
                feedback = base_cyclic.RefreshFeedback()
                position = [feedback.base.tool_pose_x, feedback.base.tool_pose_y, feedback.base.tool_pose_z,
                            feedback.base.tool_pose_theta_x, feedback.base.tool_pose_theta_y, feedback.base.tool_pose_theta_z]
                return position
            except KServerException as ex:
                print("Unable to get current tool position")
                print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
                print("Caught expected error: {}".format(ex))
                return None

    def get_joint_positions(self):
        with self.utilities.DeviceConnection.createTcpConnection(self.args) as router:
            base_cyclic = BaseCyclicClient(router)
            try:
                feedback = base_cyclic.RefreshFeedback()
                return [actuator.position for actuator in feedback.actuators]
            except KServerException as ex:
                print("Unable to get current joint positions")
                print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
                print("Caught expected error: {}".format(ex))
                return None
            
    def move_to_joint_positions(self, target_positions):
        action = Base_pb2.Action()
        action.name = "Move to target joint positions"
        action.application_data = ""

        for i, target_position in enumerate(target_positions):
            joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
            joint_angle.joint_identifier = i
            joint_angle.value = target_position

        e = threading.Event()
        with self.utilities.DeviceConnection.createTcpConnection(self.args) as router:
            base = BaseClient(router)
            notification_handle = base.OnNotificationActionTopic(
                self.check_for_end_or_abort(e),
                Base_pb2.NotificationOptions()
            )

            base.ExecuteAction(action)
            finished = e.wait(TIMEOUT_DURATION)
            base.Unsubscribe(notification_handle)

            if finished:
                print("Movement to target joint positions completed")
            else:
                print("Timeout on action notification wait")
            return finished

    def move_to_tool_position(self, target_position):
        action = Base_pb2.Action()
        action.name = "Move to target tool position"
        action.application_data = ""

        cartesian_pose = action.reach_pose.target_pose
        cartesian_pose.x = target_position["x"]
        cartesian_pose.y = target_position["y"]
        cartesian_pose.z = target_position["z"]
        cartesian_pose.theta_x = target_position["theta_x"]
        cartesian_pose.theta_y = target_position["theta_y"]
        cartesian_pose.theta_z = target_position["theta_z"]

        e = threading.Event()
        with self.utilities.DeviceConnection.createTcpConnection(self.args) as router:
            base = BaseClient(router)
            notification_handle = base.OnNotificationActionTopic(
                self.check_for_end_or_abort(e),
                Base_pb2.NotificationOptions()
            )

            base.ExecuteAction(action)
            finished = e.wait(TIMEOUT_DURATION)
            base.Unsubscribe(notification_handle)

            if finished:
                print("Movement to target position completed")
            else:
                print("Timeout on action notification wait")
            return finished
        

    # def CptIK_P(self, Input_IkData):
        


    def CptIK(self, Input_IkData):
        with self.utilities.DeviceConnection.createTcpConnection(self.args) as router:
            base = BaseClient(router)
            computed_joint_angles = base.ComputeInverseKinematics(Input_IkData)
            # print(computed_joint_angles)
            return computed_joint_angles

    # def close(self):
    #     self.router.Close()
    #     self.router_real_time.Close()

# Example usage
if __name__ == '__main__':
    robot = KinovaRobot("192.168.1.10", "admin", "admin")
    print(robot.get_tool_position())
    print(robot.get_joint_positions())
    target_position = {
        "x": 0.4,
        "y": 0.2,
        "z": 0.2,
        "theta_x": 90,
        "theta_y": 90,
        "theta_z": 90
    }
    robot.move_to_tool_position(target_position)
    # robot.close()