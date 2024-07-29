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

        # ***** DEBUG *****
        print("Final pose: ", final_pose)
        # *****************
        
        # Plan 3 trajectory pieces + grip
        # Move to a safe height
        SAFE_HEIGHT = 0.3
        target_pose_1 = copy.deepcopy(curr_state)
        target_pose_1[2] = SAFE_HEIGHT
        # Move over the final_position
        target_pose_2 = copy.deepcopy(target_pose_1)
        target_pose_2[0] = final_pose[0]
        target_pose_2[1] = final_pose[1]
        
        final_pose_hover = copy.deepcopy(final_pose)
        final_pose_hover[2] = SAFE_HEIGHT # 5 cube heights higher than the final pose
        
        # Get the 3 trajectories and the grip
        traj_1, vel_traj_1 = self.planner.plan_trajectory(curr_state, target_pose_1)
        traj_2, vel_traj_2 = self.planner.plan_trajectory(target_pose_1, target_pose_2)
        traj_3, vel_traj_3 = self.planner.plan_trajectory(target_pose_2, final_pose)
        if grip is True:
            traj_p, vel_traj_p = self.planner.pick_cube(final_pose)
        else:
            traj_p, vel_traj_p = self.planner.release_cube(final_pose)
        print("traj 4")
        traj_4, vel_traj_4 = self.planner.plan_trajectory(final_pose, final_pose_hover, debug = True)
        print(f"traj 4 {traj_4}")
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
        add_pose_3 = np.hstack([pose_setpt_3 for i in range(8)])
        traj_3 = np.concatenate((traj_3,add_pose_3),axis=0)
        add_vel_3 = np.hstack([vel_setpt_3 for i in range(8)])
        vel_traj_3 = np.concatenate((vel_traj_3,add_vel_3),axis=0)

        pose_setpt_4 = copy.deepcopy(traj_4[-8:])
        vel_setpt_4 = np.zeros_like(vel_traj_4[-3:])
        add_pose_4 = np.hstack([pose_setpt_4 for i in range(8)])
        traj_4 = np.concatenate((traj_4,add_pose_4),axis=0)
        add_vel_4 = np.hstack([vel_setpt_4 for i in range(8)])
        vel_traj_4 = np.concatenate((vel_traj_4,add_vel_4),axis=0)



        print("Requested final pose = ", final_pose)
        print("Actual final pose = ", traj_3[-8:])

        # Build unique trajectory
        traj = np.concatenate((traj_1, traj_2, traj_3, traj_p, traj_4),axis=0)
        vel_traj = np.concatenate((vel_traj_1, vel_traj_2, vel_traj_3, vel_traj_p, vel_traj_4),axis=0)

        response.completion_flag = True
        response.position = traj
        response.vel = vel_traj
        print("[INFO] trajectory generated")

        # ***** DEBUG *****

        print("traj_3 quaternions")
        for i in range(len(traj_3)//8):
            print(traj_3[i*8+3:i*8+7])
        print("\n")

        print("traj_p quaternions")
        for i in range(len(traj_p)//8):
            print(traj_p[i*8+3:i*8+7])
        print("\n")
        # *****************

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

