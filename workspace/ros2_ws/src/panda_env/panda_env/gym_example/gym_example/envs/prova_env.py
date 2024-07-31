import gymnasium as gym
from gymnasium  import error, spaces, utils
from gymnasium .utils import seeding

import os
import pybullet as p
import pybullet_data
from math import sin, cos, pi
import numpy as np
import random
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from scipy.ndimage import rotate

class PandaEnv(gym.Env):
    metadata = {'render_modes': ['human'], 'render_fps' : 60}

    def __init__(self):
        self.step_counter = 0
        #Init visualization
        p.connect(p.GUI)
        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=0, cameraTargetPosition=[0.55,-0.35,0.2])

        #Action space: cartesian pos of the EE and joint variable for both fingers 
        self.action_space = spaces.Box(np.array([-1]*11), np.array([1]*11))
        #Obs space: cartesian position of the EE and j variables of the 2 fingers
        self.observation_space = spaces.Box(np.array([-1]*9), np.array([1]*9))

        self.Height_map = None
        self.Height_map_prev = None
        self.In_hand_image = None

        self.objects = None

    def getObjStates(self):
        object_obs = {}
        for obj in self.object_states.keys():
            object_state, object_orientation = p.getBasePositionAndOrientation(self.objectUid[obj])
            object_state = list(object_state)
            object_orientation = R.from_matrix(R.from_quat(self.grip_rotation[obj]).as_matrix() @ R.from_quat(object_orientation).as_matrix()).as_quat().tolist()
            object_obs[obj] = object_state + object_orientation

        return object_obs

    def setObjects(self, objects):
        self.objects = objects
        print("Objects have been set")

    def step(self, action):
        '''Contains the logic of the environment, computes the state
           of the env after applying a given action'''
        
        position = action[:3]
        orientation = action[3:7]
        fingers = action[7]
        vel_ee = action[8:]

        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
        # Get orientation quaternion from action
        #current pose of the end effector
        currentPose = p.getLinkState(self.pandaUid, 11)

        joint_states = p.getJointStates(self.pandaUid, range(p.getNumJoints(self.pandaUid)))
        joint_infos = [p.getJointInfo(self.pandaUid, i) for i in range(p.getNumJoints(self.pandaUid))]
        #exclude the one that are not part of the dynamic
        joint_states = [j for j, i in zip(joint_states, joint_infos) if i[3] > -1]
        joint_positions = np.array([state[0] for state in joint_states])
        joint_velocities = np.array([state[1] for state in joint_states])

        jointPoses = p.calculateInverseKinematics(self.pandaUid,11,position, orientation)[0:7]
        # print("pandauid ", self.pandaUid)
        # print("joint_positions", list(joint_positions))
        # print("joint_velocities", list(joint_velocities))
        # print("ero", list(np.zeros_like(joint_positions)))
        jacobian_p, jacobian_r = p.calculateJacobian(self.pandaUid, 11, [0.0, 0.0, 0.0], list(joint_positions), list(joint_velocities), list(np.zeros_like(joint_positions)))
        P_jacobian = np.linalg.pinv(jacobian_p)
        # print("vel_ee ", vel_ee.T)
        # print("P matrix ", P_jacobian)
        jointVelocities = np.linalg.pinv(jacobian_p) @ vel_ee.T
        jointVelocities[7:9] = 0

        POSITION_GAIN = 0.8
        VELOCITY_GAIN = 1
        TIME_STEP = 1.0 / 300.0
        p.setTimeStep(TIME_STEP)
        p.setJointMotorControlArray(self.pandaUid, list(range(7))+[9,10], p.POSITION_CONTROL, list(jointPoses)+2*[fingers], list(jointVelocities), positionGains=list(np.ones_like(joint_positions)*POSITION_GAIN), velocityGains=list(np.ones_like(joint_positions)*VELOCITY_GAIN))
        # p.setJointMotorControlArray(self.pandaUid, list(range(7))+[9,10], p.POSITION_CONTROL, list(jointPoses)+2*[fingers])

        #The default timestep is 1/240 second, it can be changed using the setTimeStep
        p.stepSimulation()

        object_obs = {}
        for obj in self.objects.keys():
            # print(f"object {obj}")
            object_state, object_orientation = p.getBasePositionAndOrientation(self.objectUid[obj])
            object_state = list(object_state)
            print(f" object {obj} with orientation {object_orientation}")
            object_orientation = R.from_matrix(R.from_quat(self.grip_rotation[obj]).as_matrix() @ R.from_quat(object_orientation).as_matrix()).as_quat().tolist()
            object_obs[obj] = object_state + object_orientation

        state_robot = p.getLinkState(self.pandaUid, 11)[0]
        orientation_robot = p.getLinkState(self.pandaUid, 11)[1]
        state_fingers = (p.getJointState(self.pandaUid,9)[0], p.getJointState(self.pandaUid, 10)[0])
        if state_fingers[0] < 0.03:
            gripper = True   # closed gripper
        else:
            gripper = False  # open gripper

        reward = 0
        done = False

        self.step_counter += 1

        #Done is the end condition of the MDP, truncated is the time limit ending

        #info on the things that are not the agent
        #info = {'object_position': state_object}
        info = {}
        self.observation = state_robot + orientation_robot + state_fingers
        imgs = [self.Height_map, self.get_in_hand_image(action)]
        return np.array(self.observation).astype(np.float32), reward, done, False, info
    
    def _get_in_hand_image(self, pos, rot, _height_map):
        '''The in-hand image depends on the action executed on the last time step (at time t - 1). 
            If the last action was a PICK, then Ht is a set of heightmaps that describe the 3D 
            volume centered and aligned with the gripper when the PICK occurred. Otherwise, Ht is set to the zero
            value image.'''
        
        #parameter
        in_hand_size =  200 
        x, y = pos[0:2]
        r =  R.from_quat(rot)
        angles = r.as_euler('xyz', degrees=True)

        viewMatrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[x,y,0.05],
                                                            distance=.7,
                                                            yaw=-90 + angles[2],
                                                            pitch=-90,
                                                            roll=0,
                                                            upAxisIndex=2)
        
        projMatrix = p.computeProjectionMatrixFOV(fov=60,
                                                     aspect=1,
                                                     nearVal=0.1,
                                                     farVal=10.0)

        view_matrix = np.array(viewMatrix).reshape(4, 4).T
        proj_matrix = np.array(projMatrix).reshape(4, 4).T

        h = 720
        w = 960

        (width, height, px, depth_px, _) = p.getCameraImage(width=w,
                                              height=h,
                                              viewMatrix=viewMatrix,
                                              projectionMatrix=projMatrix,
                                              renderer=p.ER_BULLET_HARDWARE_OPENGL)

        far = 10.0
        near = 0.1
    
        depth_buffer_opengl = np.reshape(depth_px, (h,w))
        depth_opengl = far * near / (far - (far - near) * depth_buffer_opengl)

        # Convert depth_map to height_map
        camera_height = 0.75  # distance from table
        height_map = camera_height - depth_opengl

        # Pad heightmaps for grasps near the edges of the workspace
        height_map = np.pad(height_map, int(in_hand_size / 2), 'constant', constant_values=0.0)

        #Paramters used, the fov,fu, fv is needed if you are not using the projection matrix
        width = 960
        height = 720
        fov = np.pi/3 # 60 degrees

        #coordinates of the cube to pick
        world_coords = np.array([x, y, 0.05, 1.0])

        # Transform to camera coordinates
        # camera_coords = self.view_matrix @ world_coords
        camera_coords = view_matrix @ world_coords

        x_c = camera_coords[0]
        y_c = camera_coords[1]
        z_c = -camera_coords[2]

        # Image indexed by (u,v) --> u vertical and v lateral; 
        # (0,0) upper-left corner

        fu = (height/2) / np.tan(fov/2)
        fv = (width/2) / np.tan(fov/2)

        cx = width / 2
        cy = height / 2

        u = cy - fu * (y_c/z_c)
        v = cx + fv * (x_c/z_c)

        # Correct pixel position for padding
        u = u + (in_hand_size / 2)
        v = v + (in_hand_size / 2)
        
        # Get the corners of the crop
        u_min = int(u - (in_hand_size / 2))
        u_max = int(u + (in_hand_size / 2))
        v_min = int(v - (in_hand_size / 2))
        v_max = int(v + (in_hand_size / 2))

        # Crop heightmap
        crop = height_map[u_min:u_max, v_min:v_max]
        return crop

    def loadPanda(self):
        urdfRootPath=pybullet_data.getDataPath()
        rest_poses = [0,-0.215,0,-2.57,0,2.356,2.356,0.08,0.08]
        objUid = p.loadURDF(os.path.join(urdfRootPath, "franka_panda/panda.urdf"),useFixedBase=True)
        for i in range(7):
            p.resetJointState(objUid,i, rest_poses[i])
        p.resetJointState(objUid, 9, 0.08)
        p.resetJointState(objUid,10, 0.08)
        return objUid

    def loadRedCube(self, position=(0.6,0.1,0.05), orientation=(1,0,0,0)):
        urdfRootPathOurs = "/root/workspace/ros2_ws/src/panda_env/panda_env/gym_example/gym_example/envs/objects"
        urdf_path = os.path.join(urdfRootPathOurs, 'cube_red.urdf')
        objUid = p.loadURDF(urdf_path, basePosition=position, baseOrientation=orientation)
        grip_orientation = [0,0,0,1]
        return objUid, grip_orientation
    
    def loadBlueCube(self, position=(0.7,0.1,0.05), orientation=(cos(pi/16),sin(pi/16),0,0)):
        urdfRootPathOurs = "/root/workspace/ros2_ws/src/panda_env/panda_env/gym_example/gym_example/envs/objects"
        urdf_path = os.path.join(urdfRootPathOurs, 'cube_blue.urdf')
        objUid = p.loadURDF(urdf_path, basePosition=position, baseOrientation=orientation)
        grip_orientation = [0,0,0,1]
        return objUid, grip_orientation
    
    def loadGreenCube(self, position=(0.5,0.1,0.05), orientation=(cos(pi/8),sin(pi/8),0,0)):
        urdfRootPathOurs = "/root/workspace/ros2_ws/src/panda_env/panda_env/gym_example/gym_example/envs/objects"
        urdf_path = os.path.join(urdfRootPathOurs, 'cube_green.urdf')
        objUid = p.loadURDF(urdf_path, basePosition=position, baseOrientation=orientation)
        grip_orientation = [0,0,0,1]
        return objUid, grip_orientation
    
    def loadSmallCube(self, position=(0.7,0,0.05), orientation=(1,0,0,0)):
        urdfRootPathOurs = "/root/workspace/ros2_ws/src/panda_env/panda_env/gym_example/gym_example/envs/objects"
        urdf_path = os.path.join(urdfRootPathOurs, 'cube_small.urdf')
        objUid = p.loadURDF(urdf_path, basePosition=position, baseOrientation=orientation)
        grip_orientation = [0,0,0,1]
        return objUid, grip_orientation
    
    def loadCube(self, position=(0.7,0,0.05), orientation=(1,0,0,0)):
        urdfRootPathOurs = "/root/workspace/ros2_ws/src/panda_env/panda_env/gym_example/gym_example/envs/objects"
        urdf_path = os.path.join(urdfRootPathOurs, 'cube.urdf')
        objUid = p.loadURDF(urdf_path, basePosition=position, baseOrientation=orientation)
        grip_orientation = [0,0,0,1]
        return objUid, grip_orientation
    
    def loadYellowTriangle(self, position=(0.8,0.1,0.05), orientation=(1,0,0,0)):
        urdfRootPathOurs = "/root/workspace/ros2_ws/src/panda_env/panda_env/gym_example/gym_example/envs/objects"
        urdf_path = os.path.join(urdfRootPathOurs, 'triangle_yellow.urdf')
        objUid = p.loadURDF(urdf_path, basePosition=position, baseOrientation=orientation)
        grip_orientation = [-1,0,0,0]
        return objUid, grip_orientation
    
    def loadBin(self, position=(0.65,-0.1,0.05), orientation=(0,0,0,1)):
        size=(0.2, 0.2, 0.05)
        thickness=0.005

        bottom_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[size[0]/2, size[1]/2, thickness], rgbaColor=[1, 1, 1, 1])
        bottom_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size[0]/2, size[1]/2, thickness])

        front_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[thickness, size[1]/2, size[2]/2], rgbaColor=[1, 1, 1, 1])
        front_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[thickness, size[1]/2, size[2]/2])

        back_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[thickness, size[1] / 2, size[2] / 2], rgbaColor=[1, 1, 1, 1])
        back_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[thickness, size[1] / 2, size[2] / 2])

        left_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[size[0]/2, thickness, size[2] / 2], rgbaColor=[1, 1, 1, 1])
        left_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size[0]/2, thickness, size[2] / 2])

        right_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[size[0]/2, thickness, size[2]/2], rgbaColor=[1, 1, 1, 1])
        right_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size[0]/2, thickness, size[2]/2])

        bin_id = p.createMultiBody(baseMass=0,
                                    baseCollisionShapeIndex=bottom_collision,
                                    baseVisualShapeIndex=bottom_visual,
                                    basePosition=position,
                                    baseOrientation=orientation,
                                    linkMasses=[1, 1, 1, 1],
                                    linkCollisionShapeIndices=[front_collision, back_collision, left_collision, right_collision],
                                    linkVisualShapeIndices=[front_visual, back_visual, left_visual, right_visual],
                                    linkPositions=[[-size[0]/2, 0, size[2]/2],
                                                    [size[0]/2, 0, size[2]/2],
                                                    [0, -size[1]/2, size[2]/2],
                                                    [0, size[1]/2, size[2]/2]],
                                    linkOrientations=[[0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1]],
                                    linkInertialFramePositions=[[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
                                    linkInertialFrameOrientations=[[0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1]],
                                    linkParentIndices=[0, 0, 0, 0],
                                    linkJointTypes=[p.JOINT_FIXED, p.JOINT_FIXED, p.JOINT_FIXED, p.JOINT_FIXED],
                                    linkJointAxis=[[0, 0, 1], [0, 0, 1], [0, 0, 1], [0, 0, 1]]
        )

        grip_orientation = [1,0,0,0,]
        return bin_id, grip_orientation
    
    def loadBottle(self, position=(0.65,-0.1,0.05), orientation=(0,0,0,1)):
        urdfRootPathOurs = "/root/workspace/ros2_ws/src/panda_env/panda_env/gym_example/gym_example/envs/objects"
        urdf_path = os.path.join(urdfRootPathOurs, 'bottle1.urdf')
        objUid = p.loadURDF(urdf_path, basePosition=position, baseOrientation=orientation)
        grip_orientation = [1,0,0,0]
        return objUid, grip_orientation

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
        tableUid = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"),basePosition=[0.5,0,-0.65])
        # trayUid = p.loadURDF(os.path.join(urdfRootPath, "tray/traybox.urdf"),basePosition=[0.65,0,0])
        self.pandaUid = self.loadPanda()
        
        gen_funcs = {
            "red_cube" : self.loadRedCube,
            "green_cube" : self.loadGreenCube,
            "blue_cube" : self.loadBlueCube,
            "cube" : self.loadCube,
            "small_cube" : self.loadSmallCube,
            "yellow_triangle" : self.loadYellowTriangle,
            "bin" : self.loadBin,
            "bottle" : self.loadBottle
        }

        self.grip_rotation = {}
        self.objectUid = {}

        # Load objects
        if self.objects is not None:
            for obj_name, data in self.objects.items():
                self.objectUid[obj_name],self.grip_rotation[obj_name] = gen_funcs[data["type"]](position=data["position"], orientation=data["orientation"])
        else:
            # If self.objects does not exist load predefined objects
            self.objectUid["red_cube"], self.grip_rotation["red_cube"] = gen_funcs["red_cube"]()
            self.objectUid["green_cube"], self.grip_rotation["green_cube"] = gen_funcs["green_cube"]()
            self.objectUid["blue_cube"], self.grip_rotation["blue_cube"] = gen_funcs["blue_cube"]()
            self.objectUid["yellow_triangle"], self.grip_rotation["yellow_triangle"] = gen_funcs["yellow_triangle"]()
            self.objectUid["bin"], self.grip_rotation["bin"] = gen_funcs["bin"]()
            self.objectUid["bottle"], self.grip_rotation["bottle"] = gen_funcs["bottle"]()

        print("self.objectUid = ", self.objectUid)
        
        # Log object positionand orientation
        self.object_states = {}
        for obj in self.objectUid.keys():
            position, orientation = p.getBasePositionAndOrientation(self.objectUid[obj]) 
            position = list(position)
            orientation = R.from_matrix(R.from_quat(self.grip_rotation[obj]).as_matrix() @ R.from_quat(orientation).as_matrix()).as_quat().tolist()
            self.object_states[obj] = position + orientation

        print("RESET objStates = ", self.object_states)

        #we return the first observation
        state_robot = p.getLinkState(self.pandaUid, 11)[0]
        orientation_robot = p.getLinkState(self.pandaUid, 11)[1]
        state_fingers = (p.getJointState(self.pandaUid,9)[0], p.getJointState(self.pandaUid, 10)[0])
        self.observation = state_robot + orientation_robot + state_fingers
        #Now that everything is done, we can load 
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
        info = {"object_position" : self.object_states}
        return np.array(self.observation).astype(np.float32), info
    
    def render_images(self, state):
        '''
        Render height_map and in_hand image
        '''
        print("render with transparency")
        far = 10.0
        near = 0.5

        view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0.7,0,0.05],
                                                            distance=.7,
                                                            yaw=-90,
                                                            pitch=-90,
                                                            roll=0,
                                                            upAxisIndex=2)
        self.view_matrix = np.array(view_matrix).reshape(4, 4).T
        proj_matrix = p.computeProjectionMatrixFOV(fov=60,
                                                     aspect=1,
                                                     nearVal=near,
                                                     farVal=far)

        self.proj_matrix = np.array(proj_matrix).reshape(4, 4).T

        h = 720
        w = 960

        # Make the robot and the picked cube transparent
        alpha = 0.0

        print("make transparent")
        body_id = self.pandaUid
        for joint in range(p.getNumJoints(body_id) - 1):
            r, g, b, _ = p.getVisualShapeData(body_id)[joint][7]
            p.changeVisualShape(body_id, joint, rgbaColor=[r, g, b, alpha])
        r, g, b, _ = p.getVisualShapeData(body_id)[0][7]
        p.changeVisualShape(body_id, -1, rgbaColor=[r, g, b, alpha])

        # height_map
        (width, height, px, depth_px, _) = p.getCameraImage(width=w,
                                              height=h,
                                              viewMatrix=view_matrix,
                                              projectionMatrix=proj_matrix,
                                              renderer=p.ER_BULLET_HARDWARE_OPENGL)
    
        depth_buffer_opengl = np.reshape(depth_px, (h,w))
        depth_opengl = far * near / (far - (far - near) * depth_buffer_opengl)

        camera_height = 0.75
        height_map = camera_height - depth_opengl


        # in_hand_img
        pos = state[:3]
        rot = state[3:7]
        in_hand_img = self._get_in_hand_image(pos, rot, height_map)

        # Reset visibility
        alpha = 1.0

        print("reset visibility")
        body_id = self.pandaUid
        for joint in range(p.getNumJoints(body_id)-1):
            r, g, b, _ = p.getVisualShapeData(body_id)[joint][7]
            p.changeVisualShape(body_id, joint, rgbaColor=[r, g, b, alpha])
        r, g, b, _ = p.getVisualShapeData(body_id)[0][7]
        p.changeVisualShape(body_id, -1, rgbaColor=[r, g, b, alpha])

        return height_map, in_hand_img

    def render(self, mode='human'):
        #info on matrix characteristics and position
        view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0.7,0,0.05],
                                                            distance=.7,
                                                            yaw=-90,
                                                            pitch=-90,
                                                            roll=0,
                                                            upAxisIndex=2)
        self.view_matrix = np.array(view_matrix).reshape(4, 4).T
        proj_matrix = p.computeProjectionMatrixFOV(fov=60,
                                                     aspect=1,
                                                     nearVal=0.1,
                                                     farVal=10.0)

        self.proj_matrix = np.array(proj_matrix).reshape(4, 4).T

        h = 720
        w = 960

        (width, height, px, depth_px, _) = p.getCameraImage(width=w,
                                              height=h,
                                              viewMatrix=view_matrix,
                                              projectionMatrix=proj_matrix,
                                              renderer=p.ER_BULLET_HARDWARE_OPENGL)

        rgb_array = np.array(px, dtype=np.uint8)
        rgb_array = np.reshape(rgb_array, (h,w, 4))

        rgb_array = rgb_array[:, :, :3]

        far = 10.0
        near = 0.1
    
        depth_buffer_opengl = np.reshape(depth_px, (h,w))
        depth_opengl = far * near / (far - (far - near) * depth_buffer_opengl)

        camera_height = 0.75
        self.Height_map = camera_height - depth_opengl
        # print(f"dimension heigh map ", self.Height_map.shape)
        # projection_matrix_inv = np.linalg.inv(projection_matrix)
        # view_matrix_inv = np.linalg.inv(view_matrix)

        # # Convert to homogeneous coordinates
        # clip_coords = np.array([x, y, depth, 1.0])
        # eye_coords = projection_matrix_inv @ clip_coords
        # eye_coords /= eye_coords[3]
        
        # world_coords = view_matrix_inv @ eye_coords
        # world_coords /= world_coords[3]
        #depth image to publish as state 

        return rgb_array, depth_buffer_opengl


    def _get_state(self):
        return self.observation
    
    def get_in_hand_image(self, action):
        if self.Height_map_prev is not None: 
            orientation = action[3:7]
            position = action[:3]
            fingers = action[7]
            
            self.In_hand_image = self._get_in_hand_image(position, orientation, self.Height_map_prev)
            return self.In_hand_image
        else: 
            return "Not the in hand image"

    def close(self):
        p.disconnect()

