from typing import TypeVar
import numpy as np
import math
from numpy.linalg import inv

Pose = TypeVar("Pose")
Trajectory = TypeVar("Trajectory")

class PlannerInterface:
    def __init__(self):
        #grip value 0.04 = OPEN, 0 = CLOSE
        self.grip_value = 0.04
        self.TIMEOUT = 100
        self.NUM_STEPS = 20
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

        # Store trajectory in a 7xNUM_STEPS ndarray

        trajectory = np.zeros((self.NUM_STEPS, 7))
        grip_array = np.ones(((self.NUM_STEPS, 1))) * self.grip_value
        for i in range(self.NUM_STEPS):
            # Position
            trajectory[i,0] = x_y_z_theta[0][i]
            trajectory[i,1] = x_y_z_theta[1][i]
            trajectory[i,2] = x_y_z_theta[2][i]

            # Orientation
            R_theta = self._axis_to_mat(axis, x_y_z_theta[3][i])
            R = A_rot @ R_theta
            quat = self._matrix_to_quat(R)
            trajectory[i,3:] = quat.T

        #return a valid set of action for the robot
        return np.hstack((trajectory, grip_array))
    
    def pick_cube(self, A):
        '''
        A: must be the position of the cube we want to pick
        return a valid action and change the value of the gripper for the future generation
        of trajectories 
        '''
        self.grip_value = 0.02
        gripping_state = np.hstack((A, self.grip_value)) 
        return np.vstack((wait_state, gripping_state))

    def release_cube(self, A):
        '''
        A: must be the position of the cube we want to pick
        return a valid action and change the value of the gripper for the future generation
        of trajectories 
        '''
        self.grip_value = 0.04
        wait_state = np.hstack((A, 0.2))
        degripping_state = np.hstack((A, self.grip_value)) 
        return np.vstack((wait_state, degripping_state))