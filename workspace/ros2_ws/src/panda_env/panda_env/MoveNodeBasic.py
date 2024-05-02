import rclpy
from rclpy.node import Node
import copy
from robotgpt_interfaces.srv import CodeExecution, EECommands, Trajectory
from robotgpt_interfaces.msg import StateReward, Action, State
from panda_env.PlannerInterface import PlannerInterface
import numpy as np
from geometry_msgs.msg import Point
import time

class MoveNodeBasic(Node):

    def __init__(self):
        super().__init__('MoveNodeBasic')
        print("Hello from MoveNodeBasic!")

        self.EPSILON = 1e-3
        self.planner = PlannerInterface()

        env_time = 1/240
        #Service for EE trajectory generation
        self.trajectory_service = self.create_service(Trajectory, 'traj', self.traj_callback)

    def traj_callback(self, request, response):
        print("received request for trajectory")
        curr_state = request.current_position
        final_pose = request.ending_position
        grip = request.gripper_state

        curr_state = np.array(curr_state)
        final_pose = np.array(final_pose)
        traj, vel_traj = self.planner.plan_trajectory(curr_state, final_pose)
        # print("traj shape ", traj.shape)
        # print("vel_traj shape", vel_traj.shape)
        if grip is True:
            traj_p, vel_traj_p = self.planner.pick_cube(final_pose)
            traj = np.hstack((traj, traj_p))
            vel_traj_p = np.hstack((vel_traj, vel_traj_p))
        else:
            traj_p, vel_traj_p = self.planner.release_cube(final_pose)
            traj = np.hstack((traj, traj_p))
            vel_traj_p = np.hstack((vel_traj, vel_traj_p))
        response.completion_flag = True
        response.position = traj
        response.vel = vel_traj_p
        print("[INFO] trajectory generated")
        print("[INFO] response sent ----------")
        # print(response)
        print("----------------------")
        return response
    

def main(args=None):
    rclpy.init(args=args)
    node = MoveNodeBasic()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

