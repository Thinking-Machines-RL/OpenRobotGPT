import torch
import csv
from models import ResUCatShared, InHandConv, CNNShared
import copy
import matplotlib.pyplot as plt
from PIL import Image
from torchvision import transforms
import os
import torch.nn.functional as F
import random
from scipy.spatial.transform import Rotation as R
import numpy as np
import math


class Params:
    patch_size = 24
    theta_resolution = 360
    l = 1
    margin = 0
    gamma = 0.99
    w = 0.5
    M = 10
    N = 10
    lr = 0.003
    alpha = 0.01

class Agent:
    def __init__(self):
        self.q1 = ResUCatShared()
        self.q1_target = copy.deepcopy(self.q1)
        self.q2 = CNNShared((2, Params.patch_size, Params.patch_size),Params.theta_resolution)
        self.q2_target = copy.deepcopy(self.q2)
        self.optimizer_q1 = torch.optim.Adam(self.q1.parameters(),lr=Params.lr)
        self.optimizer_q2 = torch.optim.Adam(self.q2.parameters(),lr=Params.lr)

    def Q1(self, I, H, g):
        q1_map, e = self.q1(I, H)
        return q1_map[:,g,:,:], e
    
    def Q1_target(self, I, H, g):
        q1_map, e = self.q1_target(I, H)
        return q1_map[:,g,:,:], e

    def Q2(self, P, H, e):
        patch = torch.cat((P,H),1)  # one image "behind" the other (simply duplicate channels)
        q2_map = self.q2(e, patch)
        return q2_map
    
    def Q2_target(self, P, H, e):
        patch = torch.cat((P,H),1)  # one image "behind" the other (simply duplicate channels)
        q2_map = self.q2_target(e, patch)
        return q2_map
    
    def predict(self, I, H, g):
        q1_map, e = self.Q1(I, H, g)

        # Find optimal (u,v)
        q1_map_small = q1_map[0,19:109,19:109]
        uv = torch.argmax(q1_map_small)
        uv = divmod(uv.item(), q1_map_small.size(1))
        u,v = uv[0]+19,uv[1]+19

        # Crop patch
        half_patch = int(Params.patch_size/2)
        P = I[:,:,u-half_patch:u+half_patch, v-half_patch:v+half_patch]

        q2_map = self.Q2(P, H, e)

        # Find optimal theta
        j = torch.argmax(q2_map).item()
        return u,v,j

    def l(self, a: torch.Tensor, a_e: torch.Tensor) -> torch.Tensor:
        '''the non expert action penalty
        a: tensor with actions of the agent
        a_e: tensor with actions of the expert
        
        return l: the loss of the non expert
        '''
        # large margin loss penalty for non-expert actions
        # https://arxiv.org/pdf/1704.03732
        l = Params.l * torch.where(a == a_e, 0, 1)
        return l

    def lossTD1(self, s: tuple, a: torch.Tensor, y: torch.Tensor) -> torch.Tensor:
        '''calculate the first TD loss for Q1
        s: it's a tuple of 3 dimension containing 3 tensor, one for the 
            height_images, one for the in hand images and one for the
            boolean related to the gripper status
            
        a: tensor of actions, which x,y pixels we want to know the Q value
        y: TD target
        
        return:
            the TD loss for Q1
            the embeding e
        '''
        q1_map, e = self.Q1(s[0], s[1], s[2])
        u = a[:,0]
        v = a[:,1]
        idx = torch.arange(q1_map.size(0))

        print("u = ", u)
        print("v = ", v)
        print("idx = ", idx)

        Q1_pred = q1_map[idx, u[idx], v[idx]]
        loss = F.huber_loss(Q1_pred, y)
        return loss, e

    def lossTD2(self, s: tuple, a: torch.Tensor, y: torch.Tensor) -> torch.Tensor:
        '''calculate the first TD loss for Q2
        s: [P, H, e]
        
        e: tensor for encoding
            
        a: tensor of actions
        y: TD target
        
        return the TD loss for Q2
        '''
        q2_map = self.Q2(s[0], s[1], s[2])
        j = a[:,0]
        idx = torch.arange(q2_map.size(0))
        Q2_pred = q2_map[idx, j[idx]]
        loss = F.huber_loss(Q2_pred, y)
        return loss
    
    def lossSLM(self, s: tuple, a_e: torch.Tensor) -> torch.Tensor:
        # Compute q1_map
        q1_map, e = self.Q1(s[0],s[1],s[2])
        q1_a_e = q1_map[0,a_e[0][0],a_e[0][1]]
        l = Params.l * torch.ones((q1_map.size(0), q1_map.size(1)))
        l[:,a_e[0][0],a_e[0][1]] = 0
        loss_SLM1 = torch.mean(torch.ReLu(q1_map + l - q1_a_e))
        
        # Compute q2_map
        u, v = a_e[0][0], a_e[0][1]
        half_patch = int(Params.patch_size/2)
        P = s[0][u-half_patch:u+half_patch, v-half_patch:v+half_patch]
        q2_map = self.Q2(P, s[1], e)
        q2_a_e = q2_map[0,a_e[0][2]]
        l = Params.l * torch.ones((q2_map.size(0), q2_map.size(1)))
        l[:,a_e[0][2]] = 0
        loss_SLM2 = torch.mean(torch.ReLu(q2_map + l - q2_a_e))

        return loss_SLM1 + loss_SLM2

    """
    def lossSLM(self, s: tuple, a_e: torch.Tensor) -> torch.Tensor:
        '''calculate the SLM loss
        s: it's a tuple of 3 dimension containing 3 tensor, one for the 
            height_images, one for the in hand images and one for the
            boolean related to the gripper status
            
        a_e: tensor of expert actions
        y: TD target
        
        return the TD loss for Q2
        '''

        # Calculating the strict large margin loss (SLM)
        u = s[0].size(1)
        v = s[0].size(2)
        n = s[0].size(0) # aks: in theory should be 1?
        u_coords, v_coords = torch.meshgrid(torch.arange(u), torch.arange(v), indexing='ij')
        action_q1 = torch.stack((u_coords, v_coords), dim=-1).reshape(-1, 2) #(UV)x2
        action_q1 = action_q1.unsqueeze(1) #dimension (UV)x1x2
        ae1 = a_e[0:2].unsqueeze(0) #dimension 1x(N)x2
        #TODO: should be equal in norm or what? can I just sum x+y?
        margin_loss_1 = self.l(action_q1, a_e[0,0:2]) #dimension loss((UV)x1x2 - 1xNx2) = Relu((uv)x(n)x2) = (UV)xN
        q1_map, e = self.Q1(s[0], s[1], s[2]) #q1_map = tensor nxuxv
        #NxUxV > (1xN - (UXV)xN) = UxVxN > (UV)xN
        #case N = 1 > UxV > 1 - UV
        q1_map_reshape = q1_map.reshape(n, -1)
        a1 = a_e[:,0]
        a2 = a_e[:,1]
        q1_e = q1_map[torch.arange(q1_map.size(0)), a1, a2].reshape(n, -1)
        filter1 = q1_map_reshape[q1_map_reshape > q1_e] #(NxUV)

        q2_map = self.Q2(s[0], s[1], s[2], e, a_e[:,0:2]) #dimension Nx180
        action_q2 = torch.arange(q2_map.size(1)) + 0.5 #dimension Nx180
        ae3 = a_e[0:2]#dimension 1x(N)
        margin_loss_2 = self.l(action_q2, ae3) 
        q2_e = q2_map[torch.arange(q1_map.size(0)), int(ae3)] #Nx1
        filter2 = q2_map[q2_map > q2_e] #Nx180 > Nx1
        #q1 only values in the range
        LSLM = torch.mean(q1_map_reshape[filter1] + margin_loss_1.transpose(0,1)[filter1] -  q1_e)
        LSLM += torch.mean(q2_map[filter2] + margin_loss_2 -  q2_e)
        return LSLM
    """

    def _sample(self, dataset):
        n = len(dataset)
        idx = random.randint(0,n-1)
        return dataset[idx]
    
    def _soft_update(self, target_q_network, q_network):
        for target_param, param in zip(target_q_network.parameters(), q_network.parameters()):
            target_param.data.copy_((1 - Params.alpha) * target_param.data + Params.alpha * param.data)
    

    def pre_train(self, De: torch.Tensor):
        #DE = list of tuple of the transition
        # S = list

        for _ in range(Params.M):
            D = self._sample(De)
            #D is a tuple
            #Pre_training for M
            s = D[0]
            I = s[0]
            H = s[1]
            g = 1 if s[2] else 0

            a_e = torch.tensor(D[1][0:3]).unsqueeze(0)
        
            sn = D[3]
            In = sn[0]
            Hn = sn[1]
            gn = 1 if sn[2] else 0

            r = D[2][0]

            with torch.no_grad():
                #a1 are the x,y coordinates
                q1_map, e = self.Q1_target(In, Hn, gn)
                q1_map_small = q1_map[0,19:109,19:109]
                a1_n = torch.argmax(q1_map_small)
                uv = divmod(a1_n.item(), q1_map_small.size(1))
                u,v = uv[0]+19,uv[1]+19

                print("u = ", u)
                print("v = ", v)

                # Crop patch
                half_patch = int(Params.patch_size/2)
                P = In[:,:,u-half_patch:u+half_patch, v-half_patch:v+half_patch]
                print("P.shape = ", P.shape)
                print("H.shape = ", H.shape)
                q2_map = self.Q2_target(P, Hn, e)
                a2_n = torch.argmax(q2_map)
                y = r + Params.gamma*torch.max(q2_map)

            q1_map, e = self.Q1(I, H, g)
            q1_map_small = q1_map[0,19:109,19:109]
            a1 = torch.argmax(q1_map_small)
            uv = divmod(a1.item(), q1_map_small.size(1))
            u,v = uv[0]+19,uv[1]+19

            print("u = ", u)
            print("v = ", v)

            # Crop patch
            half_patch = int(Params.patch_size/2)
            P = I[:,:,u-half_patch:u+half_patch, v-half_patch:v+half_patch]
            print("P.shape = ", P.shape)
            print("H.shape = ", H.shape)
            q2_map = self.Q2_target(P, H, e)
            a2 = torch.argmax(q2_map)

            s = tuple([I, H, g])
            loss_TD1, e = self.lossTD1(s, a_e[:,0:2], y)

            print("a_e = ", a_e)

            P = I[:,:,a_e[0][0].item()-half_patch:a_e[0][0].item()+half_patch, a_e[0][1].item()-half_patch:a_e[0][1].item()+half_patch]
            loss_TD2 = self.lossTD2([P,H,e], a_e[:,2:], y)

            loss_TD =  loss_TD1 + loss_TD2
            loss_SLM = self.lossSLM(s, a_e)
            # w = something
            L = loss_TD + Params.w* loss_SLM
            # gradient descent
            #Fist Q1
            self.optimizer_q1.zero_grad()
            L.backward(retain_graph=True)
            self.optimizer_q1.step()
            #Second Q2
            self.optimizer_q2.zero_grad()
            L.backward(retain_graph=True)
            self.optimizer_q2.step()

            # Soft update
            self._soft_update(self.q1_target, self.q1)
            self._soft_update(self.q2_target, self.q2)

    
    def train(self, De, D):
        # We want to sample half transitions from De and half from D
        # We use two separete cycles

        # Expert transitions
        for i in range (Params.N/2):
            s, a, r, ns = self._sample(De)

            with torch.no_grad():
                # Compute optimal action from next state
                q1_map, e = self.Q1_target(ns[0], ns[1], ns[2])
                q1_map_small = q1_map[0,19:109,19:109]
                uv = torch.argmax(q1_map)
                uv = divmod(uv.item(), q1_map_small.size(1))
                u,v = uv[0]+19,uv[1]+19

                # Crop patch
                half_patch = int(Params.patch_size/2)
                P = ns[0][:,:,u-half_patch:u+half_patch, v-half_patch:v+half_patch]
                q2_map = self.Q2_target(P, ns[1], e)
                j = torch.argmax(q2_map).item()
                y = r[0] + Params.gamma * torch.max(q2_map)



            loss_TD1, e = self.lossTD1(s, a[0:2], y)
            s2 = [s[:,:,a[0]-half_patch:a[0]+half_patch, a[1]-half_patch:a[1]+half_patch]]
            loss_TD2 = self.lossTD2([s2,s[1],e], a[2], y)
            loss_TD =  loss_TD1 + loss_TD2
            loss_SLM = self.lossSLM(s, a)

            L = loss_TD + Params.w* loss_SLM

            # Gradient descent step
            self.optimizer_q1.zero_grad()
            L.backward(retain_graph=True)
            self.optimizer_q1.step()
            #Second Q2
            self.optimizer_q2.zero_grad()
            L.backward(retain_graph=True)
            self.optimizer_q2.step()

            # Update target network
            self._soft_update(self.q1_target, self.q1)
            self._soft_update(self.q2_target, self.q2)
            

        # Acquired transitions
        for i in range (Params.N/2):
            s, a, r, ns = self._sample(D)

            with torch.no_grad():
                # Compute optimal action from next state
                q1_map, e = self.Q1_target(ns[0], ns[1], ns[2])
                q1_map_small = q1_map[0,19:109,19:109]
                uv = torch.argmax(q1_map)
                uv = divmod(uv.item(), q1_map_small.size(1))
                u,v = uv[0]+19,uv[1]+19

                # Crop patch
                half_patch = int(Params.patch_size/2)
                P = ns[0][:,:,u-half_patch:u+half_patch, v-half_patch:v+half_patch]
                q2_map = self.Q2_target(P, ns[1], e)
                j = torch.argmax(q2_map).item()
                y = r[0] + Params.gamma * torch.max(q2_map)



            loss_TD1, e = self.lossTD1(s, a[0:2], y)
            s2 = [s[:,:,a[0]-half_patch:a[0]+half_patch, a[1]-half_patch:a[1]+half_patch]]
            loss_TD2 = self.lossTD2([s2,s[1],e], a[2], y)
            loss_TD =  loss_TD1 + loss_TD2

            L = loss_TD

            # Gradient descent step
            self.optimizer_q1.zero_grad()
            L.backward(retain_graph=True)
            self.optimizer_q1.step()
            #Second Q2
            self.optimizer_q2.zero_grad()
            L.backward(retain_graph=True)
            self.optimizer_q2.step()

            # Update target network
            self._soft_update(self.q1_target, self.q1)
            self._soft_update(self.q2_target, self.q2)



def read_csv(path):
    with open(path, 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        data = []
        for row in csvreader:
            data.append(row)
    return data[1:]


def loadD(path):
    conv_factor = os.path.join(path,"conv_factor.csv")
    states_path = os.path.join(path,"states.csv")
    actions_path = os.path.join(path,"actions.csv")
    rewards_path = os.path.join(path,"rewards.csv")
    next_states_path = os.path.join(path,"next_states.csv")

    conv_factor = read_csv(conv_factor)[0][0]
    states = read_csv(states_path)
    actions = read_csv(actions_path)
    rewards = read_csv(rewards_path)
    next_states = read_csv(next_states_path)

    bool_conversion = {'True': True, 'False': False}

    for state in states:
        path_height_map = os.path.join(path, "imgs", state[0])
        path_in_hand_img = os.path.join(path, "imgs", state[1])

        height_map = Image.open(path_height_map)
        in_hand_img = Image.open(path_in_hand_img)

        resize_patch = transforms.Resize((Params.patch_size, Params.patch_size))
        resize_img = transforms.Resize((90,90))
        to_tensor = transforms.ToTensor()

        height_map = resize_img(height_map)
        in_hand_img = resize_patch(in_hand_img)

        height_map = to_tensor(height_map)[0,:,:]
        in_hand_img = to_tensor(in_hand_img)[0,:,:]

        # Pad height_map to 128x128
        I = torch.zeros((128, 128))
        I[19:109,19:109] = height_map
        height_map = I

        height_map = height_map.unsqueeze(0).unsqueeze(0)
        in_hand_img = in_hand_img.unsqueeze(0).unsqueeze(0)

        state[0] = height_map
        state[1]= in_hand_img
        state[-1] = bool_conversion[state[-1]]

    conv_factor = float(conv_factor)
    L_pixel = 90
    L_meters = conv_factor * L_pixel
    for action in actions:
        u = math.floor((L_meters - float(action[0])) / conv_factor)
        v = math.floor(float(action[1]) / conv_factor)
        u = min(u,90-1)
        v = min(v,90-1)
        # Create a Rotation object from the quaternion
        rotation = R.from_quat(np.array([float(a) for a in action[3:7]]))
        # Convert to angle-axis representation
        angle_axis = rotation.as_rotvec()
        # The angle of rotation (magnitude of the rotation vector)
        theta_abs = math.floor(np.linalg.norm(angle_axis) / (2*math.pi/Params.theta_resolution))
        theta = theta_abs if angle_axis[2] > 0 else (2*math.pi-theta_abs)
        theta = min(theta,Params.theta_resolution-1)

        action[0] = int(u)
        action[1] = int(v)
        action[2] = int(theta)
        for i in range(5):
            action.pop()

    for reward in rewards:
        for i in range(len(reward)):
            reward[i] = float(reward[i])

    for next_state in next_states:
        path_height_map = os.path.join(path, "imgs", next_state[0])
        path_in_hand_img = os.path.join(path, "imgs", next_state[1])

        height_map = Image.open(path_height_map)
        in_hand_img = Image.open(path_in_hand_img)

        resize_patch = transforms.Resize((Params.patch_size, Params.patch_size))
        resize_img = transforms.Resize((90,90))
        to_tensor = transforms.ToTensor()

        height_map = resize_img(height_map)
        in_hand_img = resize_patch(in_hand_img)

        height_map = to_tensor(height_map)[0,:,:]
        in_hand_img = to_tensor(in_hand_img)[0,:,:]

        # Pad height_map to 128x128
        I = torch.zeros((128, 128))
        I[19:109,19:109] = height_map
        height_map = I

        height_map = height_map.unsqueeze(0).unsqueeze(0)
        in_hand_img = in_hand_img.unsqueeze(0).unsqueeze(0)

        next_state[0] = height_map
        next_state[1]= in_hand_img
        next_state[-1] = bool_conversion[next_state[-1]]

    return list(zip(states,actions,rewards,next_states))



if __name__ == '__main__':
    agent = Agent()
    path = 'dataset/trajectory_1'

    De = loadD(path)

    agent.pre_train(De)



