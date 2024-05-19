import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
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

        traj_group = ReentrantCallbackGroup()
        env_time = 1/240
        #Service for EE trajectory generation
        self.trajectory_service = self.create_service(Trajectory, 'traj', self.traj_callback, callback_group= traj_group)

    def traj_callback(self, request, response):
        print("received request for trajectory")
        curr_state = request.current_position
        final_pose = request.ending_position
        grip = request.gripper_state

        curr_state = np.array(curr_state)
        final_pose = np.array(final_pose)
        
        # Plan 3 trajectory pieces + grip
        # Move to a safe height
        SAFE_HEIGHT = 0.3
        target_pose_1 = copy.deepcopy(curr_state)
        target_pose_1[2] = SAFE_HEIGHT
        # Move over the final_position
        target_pose_2 = copy.deepcopy(target_pose_1)
        target_pose_2[0] = final_pose[0]
        target_pose_2[1] = final_pose[1]
        
        # Get the 3 trajectories and the grip
        traj_1, vel_traj_1 = self.planner.plan_trajectory(curr_state, target_pose_1)
        traj_2, vel_traj_2 = self.planner.plan_trajectory(target_pose_1, target_pose_2)
        traj_3, vel_traj_3 = self.planner.plan_trajectory(target_pose_2, final_pose)
        if grip is True:
            traj_p, vel_traj_p = self.planner.pick_cube(final_pose)
        else:
            traj_p, vel_traj_p = self.planner.release_cube(final_pose)

        # Add extra termination points to the trajectories
        pose_setpt_1 = copy.deepcopy(traj_1[-8:])
        vel_setpt_1 = np.zeros_like(vel_traj_1[-3:])
        add_pose_1 = np.hstack([pose_setpt_1 for i in range(5)])
        traj_1 = np.concatenate((traj_1,add_pose_1),axis=0)
        add_vel_1 = np.hstack([vel_setpt_1 for i in range(5)])
        vel_traj_1 = np.concatenate((vel_traj_1,add_vel_1),axis=0)

        pose_setpt_2 = copy.deepcopy(traj_2[-8:])
        vel_setpt_2 = np.zeros_like(vel_traj_2[-3:])
        add_pose_2 = np.hstack([pose_setpt_2 for i in range(5)])
        traj_2 = np.concatenate((traj_2,add_pose_2),axis=0)
        add_vel_2 = np.hstack([vel_setpt_2 for i in range(5)])
        vel_traj_2 = np.concatenate((vel_traj_2,add_vel_2),axis=0)

        pose_setpt_3 = copy.deepcopy(traj_3[-8:])
        vel_setpt_3 = np.zeros_like(vel_traj_3[-3:])
        add_pose_3 = np.hstack([pose_setpt_3 for i in range(5)])
        traj_3 = np.concatenate((traj_3,add_pose_3),axis=0)
        add_vel_3 = np.hstack([vel_setpt_3 for i in range(5)])
        vel_traj_3 = np.concatenate((vel_traj_3,add_vel_3),axis=0)

        pose_setpt_p = copy.deepcopy(traj_p[-8:])
        vel_setpt_p = np.zeros_like(vel_traj_p[-3:])
        add_pose_p = np.hstack([pose_setpt_p for i in range(5)])
        traj_p = np.concatenate((traj_p,add_pose_p),axis=0)
        add_vel_p = np.hstack([vel_setpt_p for i in range(5)])
        vel_traj_p = np.concatenate((vel_traj_p,add_vel_p),axis=0)

        # Build unique trajectory
        traj = np.concatenate((traj_1, traj_2, traj_3, traj_p),axis=0)
        vel_traj = np.concatenate((vel_traj_1, vel_traj_2, vel_traj_3, vel_traj_p),axis=0)

        response.completion_flag = True
        response.position = traj
        response.vel = vel_traj
        print("[INFO] trajectory generated")
        print("[INFO] response sent ----------")
        # print(response)
        print("----------------------")
        return response
    

def main(args=None):
    rclpy.init(args=args)
    node = MoveNodeBasic()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

