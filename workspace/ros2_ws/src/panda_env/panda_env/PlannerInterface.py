from typing import TypeVar
import numpy as np
import math
from numpy.linalg import inv
from collections import deque

Pose = TypeVar("Pose")
Trajectory = TypeVar("Trajectory")

class PlannerInterface:
    def __init__(self):
        #grip value 0.04 = OPEN, 0 = CLOSE
        self.grip_value = 0.04
        self.TIMEOUT = 100
        self.NUM_STEPS = 40 #must be an even number
        self.eps = 1e-3
        #calculate time
        K = 1/240

    def set_pose(self, A: np.ndarray):
        #A is a np.array of size 7 [x,y,z, quaternion]
        self.curr_pose = A

    def _skew_simm(self, axis:np.ndarray):
        #axis is a np.array of size 3
        M = np.array([[0, -axis[2], axis[1]],
                      [axis[2], 0, -axis[0]],
                      [-axis[1], axis[0], 0]])
        return M
    
    def _axis_to_mat(self, axis:np.ndarray, angle)-> np.ndarray:
        #from rotation axis to rotation matrix
        S = self._skew_simm(axis)
        R = np.eye(3) + math.sin(angle)*S + (1-math.cos(angle))*(S @ S)
        return R
    
    def _mat_to_axis(self, R):
        diag_sum = R[0,0] + R[1,1] + R[2,2]
        if diag_sum > 3:
            diag_sum = 3
        angle = math.acos((diag_sum - 1)/2)
        axis = (1/2*(math.sin(angle) + self.eps)) * np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R [0,1]])
        return axis, angle
    
    def _quat_to_axis(self, quat:np.ndarray):
        angle = 2*math.acos(quat[0])
        axis = quat[1:]/(math.sin(angle/2) + self.eps)
        return axis, angle
    
    def _quat_to_mat(self, quat:np.ndarray) -> np.ndarray:
        axis, angle = self._quat_to_axis(quat)
        R = self._axis_to_mat(axis, angle)
        return R
    
    def _axis_to_quat(self, axis:np.ndarray, angle:float) -> np.ndarray:
        v = math.sin(angle/2) * axis
        quat = np.concatenate(([math.cos(angle/2)], v))
        return quat
    
    def _matrix_to_quat(self, R:np.ndarray):
        axis, angle = self._mat_to_axis(R)
        quat = self._axis_to_quat(axis, angle)
        return quat

    def plan_trajectory(self, A: np.array, B: np.array):
        print("[INFO] calculating the trajectory")
        # Plan direct path from A to B
        A_rot = self._quat_to_mat(A[3:])
        B_rot = self._quat_to_mat(B[3:])
        AB_rot = inv(A_rot) @ B_rot

        axis, theta_f = self._mat_to_axis(AB_rot)

        x_y_z_theta_0 = np.concatenate((A[:3],np.array([0])))
        x_y_z_theta_f = np.concatenate((B[:3],np.array([theta_f])))
        x_y_z_theta = [np.linspace(start, stop, num=self.NUM_STEPS) for start, stop in zip(x_y_z_theta_0, x_y_z_theta_f)]
        
        vel_in = np.linspace(0, 1.5, num = int(self.NUM_STEPS/2))
        vel_fin = np.linspace(1.5, 0, num = int(self.NUM_STEPS/2))
        vel = np.hstack((vel_in, vel_fin))
        vel_array = np.hstack((vel, vel))
        vel_array = np.hstack((vel_array, vel))
        # Store trajectory in a 7xNUM_STEPS ndarray

        trajectory = None
        grip = np.ones(((1))) * self.grip_value
        for i in range(self.NUM_STEPS):
            # Position
            position = np.array([x_y_z_theta[0][i], x_y_z_theta[1][i], x_y_z_theta[2][i]])

            # Orientation
            R_theta = self._axis_to_mat(axis, x_y_z_theta[3][i])
            R = A_rot @ R_theta
            quat = self._matrix_to_quat(R)

            orientation = quat.T
            traj_point = np.concatenate((position, orientation, grip))
            if trajectory is not None:
                trajectory = np.hstack((trajectory, traj_point))
            else:
                trajectory = traj_point

        #return a valid set of action for the robot
        return trajectory, vel_array

# def _calculate_shortest_time(distance):
#     # Constants
#     max_velocity = 1.7  # m/s
#     max_acceleration = 13.0  # m/s^2

#     # Calculate time for acceleration and deceleration phases
#     acceleration_time = max_velocity / max_acceleration #Ta
    
#     # Total time
#     total_time = distance / max_velocity + acceleration_time
#     return total_time

# def plan_trajectory(self, A: np.array, B: np.array):
#     # Constants
#     max_velocity = 1.7  # m/s
#     max_acceleration = 13.0  # m/s^2

#     distance = np.linalg.norm(A[0:3] - B[0:3])
#     total_time = calculate_shortest_time(distance)

#     # Calculate number of timesteps
#     num_steps = int(total_time / dt)

#     # Time array
#     t = np.linspace(0, total_time, num_steps)

#     # Generate trapezoidal velocity profile
#     acceleration_time = max_velocity / max_acceleration
#     distance_acceleration = 0.5 * max_acceleration * acceleration_time ** 2
#     constant_velocity_time = (total_time - 2 * acceleration_time)
#     distance_constant_velocity = max_velocity * constant_velocity_time
#     total_distance = 2 * distance_acceleration + distance_constant_velocity

#     # Initialize arrays for position, orientation, and linear velocity
#     position = np.zeros((3, num_steps))
#     orientation = np.zeros((4, num_steps))  # Using quaternion for orientation
#     linear_velocity = np.zeros((3, num_steps))

#     # Generate trajectory
#     current_pos = 0
#     for i in range(num_steps):
#         if t[i] < acceleration_time:
#             current_pos += 0.5 * max_acceleration * (t[i] ** 2)
#             linear_velocity[0, i] = max_acceleration * t[i]
#         elif t[i] < acceleration_time + constant_velocity_time:
#             current_pos += max_velocity * dt
#             linear_velocity[0, i] = max_velocity
#         else:
#             current_pos += max_velocity * (total_time - t[i]) - 0.5 * max_acceleration * (
#                     total_time - t[i]) ** 2
#             linear_velocity[0, i] = max_acceleration * (total_time - t[i])

#         position[0, i] = current_pos

#     return position, orientation, linear_velocity

    
    def pick_cube(self, A):
        '''
        A: must be the position of the cube we want to pick
        return a valid action and change the value of the gripper for the future generation
        of trajectories 
        '''
        print("Calculating the cube pick")
        self.grip_value = 0.02
        gripping_state = np.hstack((A, self.grip_value)) 
        vel_traj = np.array([0.0, 0.0, 0.0])
        gp_state = gripping_state
        for i in range(5):
            gp_state = np.hstack((gripping_state, gp_state))
            vel_traj = np.hstack((vel_traj,np.array([0.0, 0.0, 0.0])))
        return gp_state, vel_traj

    def release_cube(self, A):
        '''
        A: must be the position of the cube we want to pick
        return a valid action and change the value of the gripper for the future generation
        of trajectories 
        '''
        self.grip_value = 0.04
        degripping_state = np.hstack((A, self.grip_value)) 
        vel_traj = np.array([0.0, 0.0, 0.0])
        gp_state = degripping_state
        for i in range(5):
            gp_state = np.hstack((degripping_state, gp_state))
            vel_traj = np.hstack((vel_traj,np.array([0.0, 0.0, 0.0])))
        return gp_state, vel_traj