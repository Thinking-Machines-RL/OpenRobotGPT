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
        self.NUM_STEPS = 20 #must be an even number
        self.eps = 1e-3

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
        angle = math.acos((R[0,0] + R[1,1] + R[2,2] - 1)/2)
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
        # Plan direct path from A to B
        A_rot = self._quat_to_mat(A[3:])
        B_rot = self._quat_to_mat(A[3:])
        AB_rot = inv(A_rot) @ B_rot

        axis, theta_f = self._mat_to_axis(AB_rot)

        x_y_z_theta_0 = np.concatenate((A[:3],np.array([0])))
        x_y_z_theta_f = np.concatenate((B[:3],np.array([theta_f])))
        x_y_z_theta = [np.linspace(start, stop, num=self.NUM_STEPS) for start, stop in zip(x_y_z_theta_0, x_y_z_theta_f)]
        
        vel_in = np.linspace(0, 2, num = int(self.NUM_STEPS/2))
        vel_fin = np.linspace(2, 0, num = int(self.NUM_STEPS/2))
        vel = np.hstack((vel_in, vel_fin))
        vel_array = np.vstack((vel, vel))
        vel_array = np.vstack((vel_array, vel))
        # Store trajectory in a 7xNUM_STEPS ndarray

        trajectory = deque()
        vel_deq = deque()
        grip = np.ones(((1))) * self.grip_value
        for i in range(self.NUM_STEPS):
            # Position
            position = np.array([x_y_z_theta[0][i], x_y_z_theta[1][i], x_y_z_theta[2][i]])

            # Orientation
            R_theta = self._axis_to_mat(axis, x_y_z_theta[3][i])
            R = A_rot @ R_theta
            quat = self._matrix_to_quat(R)
            print("quat ", quat)

            orientation = quat.T
            traj_point = np.concatenate((position, orientation, grip))
            trajectory += deque([traj_point])
            vel_t = vel_array[:,i].T
            print("vel t", vel_t)
            vel_deq += deque([vel_array[:,i].T])

        #return a valid set of action for the robot
        return trajectory, vel_deq
    
    def pick_cube(self, A):
        '''
        A: must be the position of the cube we want to pick
        return a valid action and change the value of the gripper for the future generation
        of trajectories 
        '''
        self.grip_value = 0.02
        gripping_state = np.hstack((A, self.grip_value)) 
        vel_traj = deque([np.array([0.0, 0.0, 0.0])])
        gp_state = deque([gripping_state])
        for i in range(5):
            gp_state += deque([gripping_state])
            vel_traj += deque([np.array([0.0, 0.0, 0.0])])
        return gp_state, vel_traj

    def release_cube(self, A):
        '''
        A: must be the position of the cube we want to pick
        return a valid action and change the value of the gripper for the future generation
        of trajectories 
        '''
        self.grip_value = 0.04
        degripping_state = np.hstack((A, self.grip_value))
        vel_traj = deque([np.array([0.0, 0.0, 0.0])])
        dgp_state = deque([degripping_state])
        for i in range(5):
            dgp_state += deque([degripping_state])
            vel_traj += deque([np.array([0.0, 0.0, 0.0])])
        return dgp_state, vel_traj