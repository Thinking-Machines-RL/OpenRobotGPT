import gymnasium as gym
from gymnasium  import error, spaces, utils
from gymnasium .utils import seeding

import os
import pybullet as p
import pybullet_data
import math
import numpy as np
import random
import matplotlib.pyplot as plt

class PandaEnv(gym.Env):
    metadata = {'render_modes': ['human'], 'render_fps' : 60}

    def __init__(self):
        self.step_counter = 0
        #Init visualization
        p.connect(p.GUI)
        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0.55,-0.35,0.2])

        #Action space: cartesian pos of the EE and joint variable for both fingers 
        self.action_space = spaces.Box(np.array([-1]*11), np.array([1]*11))
        #Obs space: cartesian position of the EE and j variables of the 2 fingers
        self.observation_space = spaces.Box(np.array([-1]*9), np.array([1]*9))
        #variable used to update the moveit simulation
        self.joint_states = None

    def getObjStates(self):
        object_obs = {}
        for object in self.objects.keys():
            object_state, _ = p.getBasePositionAndOrientation(self.objectUid[object])
            object_obs[object] = object_state

        return object_obs

    def step(self, action):
        '''Contains the logic of the environment, computes the state
           of the env after applying a given action'''
        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)

        # Joint names as list of strings
        jointNames = action[0]
        # Joint positions and velocities 
        jointPositions = action[1].positions
        jointVelocities = action[1].velocities

        print("jointPositions = ", jointPositions)
        print("jointVelocities = ", jointVelocities)

        # TODO: linkIndex should probably depend on the joint names
        # linkIndex = list(range(7))+[9,10]
        linkIndex = list(range(7))

        p.setJointMotorControlArray(self.pandaUid, linkIndex, p.POSITION_CONTROL, jointPositions, jointVelocities, positionGains=list(np.ones_like(jointPositions)*1), velocityGains=list(np.ones_like(jointVelocities)*0.5))

        #The default timestep is 1/240 second, it can be changed using the setTimeStep
        p.stepSimulation()

        object_obs = {}
        for object in self.objects.keys():
            state_object, _ = p.getBasePositionAndOrientation(self.objectUid[object])
            object_obs[object] = state_object
        state_robot = p.getLinkState(self.pandaUid, 11)[0]
        orientation_robot = p.getLinkState(self.pandaUid, 11)[1]
        state_fingers = (p.getJointState(self.pandaUid,9)[0], p.getJointState(self.pandaUid, 10)[0])

        #TODO: random goal, must be changed
        #we moved the object at a certain altitude 
        if object_obs["blue_cube"][-1]>0.45:
            reward = 1
            done = True
        else:
            reward = 0
            done = False

        self.step_counter += 1

        #Done is the end condition of the MDP, truncated is the time limit ending

        #info on the things that are not the agent
        info = {'object_position': state_object}
        self.observation = state_robot + orientation_robot + state_fingers
        #time limit is handled by the time wrapper
        return np.array(self.observation).astype(np.float32), reward, done, False, info

    def reset(self, seed=None, options=None):
        '''This method will be called to initiate a new episode.
           You may assume that the step method will not be called before
           the reset has been called'''
        # We need the following line to seed self.np_random
        super().reset(seed=seed)
        self.step_counter = 0
        #Reset the pybullet simulation
        p.resetSimulation()
        # we will enable rendering after we loaded everything
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
        urdfRootPath=pybullet_data.getDataPath()
        p.setGravity(0,0,-10)

        #We load all the objects
        planeUid = p.loadURDF(os.path.join(urdfRootPath,"plane.urdf"), basePosition=[0,0,-0.65])

        rest_poses = [0,-0.215,0,-2.57,0,2.356,2.356,0.08,0.08]
        self.pandaUid = p.loadURDF(os.path.join(urdfRootPath, "franka_panda/panda.urdf"),useFixedBase=True)
        for i in range(7):
            p.resetJointState(self.pandaUid,i, rest_poses[i])
        p.resetJointState(self.pandaUid, 9, 0.08)
        p.resetJointState(self.pandaUid,10, 0.08)
        tableUid = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"),basePosition=[0.5,0,-0.65])

        trayUid = p.loadURDF(os.path.join(urdfRootPath, "tray/traybox.urdf"),basePosition=[0.65,0,0])
        
        #we randomize the position of the object on the table
        # state_object= [random.uniform(0.5,0.8),random.uniform(-0.2,0.2),0.05]
        # state_object= [0.6,0.1,0.05]
        # self.objectUid = p.loadURDF(os.path.join(urdfRootPath, "cube_small.urdf"), basePosition=state_object)
        objects = {"blue_cube":[0.6,0.1,0.05],
                   "yellow_cube":[0.5,-0.1,0.05],
                   "green_cube": [0.7,0.1,0.05]}
        self.objectUid = {}
        for object,position in zip(objects.keys(), objects.values()):
            self.objectUid[object] = p.loadURDF(os.path.join(urdfRootPath, "cube_small.urdf"), basePosition=position)
        
        self.objects = {}
        for obj in self.objectUid.keys():
            self.objects[obj], _ = p.getBasePositionAndOrientation(self.objectUid[obj])

        #update the moveit simulation
        joint_states = p.getJointStates(self.pandaUid, range(p.getNumJoints(self.pandaUid)))
        joint_infos = [p.getJointInfo(self.pandaUid, i) for i in range(p.getNumJoints(self.pandaUid))]
        #exclude the one that are not part of the dynamic
        joint_states = [j for j, i in zip(joint_states, joint_infos) if i[3] > -1]
        joint_states_np = np.zeros((3, len(joint_states)))
        for i, state in enumerate(joint_states):
            joint_states_np[0,i] = state[0]
            joint_states_np[1,i] = state[1]
            joint_states_np[2,i] = state[3]
        self.joint_states = joint_states_np
        
        self.joint_names = [i[1].decode('utf-8') for i in joint_infos if i[3] > -1]
        #we return the first observation
        state_robot = p.getLinkState(self.pandaUid, 11)[0]
        orientation_robot = p.getLinkState(self.pandaUid, 11)[1]
        state_fingers = (p.getJointState(self.pandaUid,9)[0], p.getJointState(self.pandaUid, 10)[0])
        self.observation = state_robot + orientation_robot + state_fingers
        #Now that everything is done, we can load 
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
        info = {"object_position" : objects}
        return np.array(self.observation).astype(np.float32), info

    def render(self, mode='human'):
        #info on matrix characteristics and position
        view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0.7,0,0.05],
                                                            distance=.7,
                                                            yaw=90,
                                                            pitch=-70,
                                                            roll=0,
                                                            upAxisIndex=2)
        proj_matrix = p.computeProjectionMatrixFOV(fov=60,
                                                     aspect=float(960) /720,
                                                     nearVal=0.1,
                                                     farVal=10.0)
        (_, _, px, depth_px, _) = p.getCameraImage(width=960,
                                              height=720,
                                              viewMatrix=view_matrix,
                                              projectionMatrix=proj_matrix,
                                              renderer=p.ER_BULLET_HARDWARE_OPENGL)

        rgb_array = np.array(px, dtype=np.uint8)
        rgb_array = np.reshape(rgb_array, (720,960, 4))

        rgb_array = rgb_array[:, :, :3]

        far = 10.0
        near = 0.1
        depth_buffer_opengl = np.reshape(depth_px, (720,960))
        depth_opengl = far * near / (far - (far - near) * depth_buffer_opengl)
        #depth image to publish as state 

        return rgb_array, depth_buffer_opengl


    def _get_state(self):
        return self.observation
    
    def get_joint_states(self):
        #function to be called by the GymNode to update the simulation to the starting value of the robot
        return self.joint_names, self.joint_states

    def close(self):
        p.disconnect()