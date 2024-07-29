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
from torch.nn.functional import relu


class Params:
    patch_size = 24
    theta_n = 360
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
        self.q2 = CNNShared((2, Params.patch_size, Params.patch_size),Params.theta_n)
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
        l = Params.l * torch.ones((q1_map.size(1), q1_map.size(2)))
        l[a_e[0][0],a_e[0][1]] = 0
        loss_SLM1 = torch.mean(relu(q1_map + l - q1_a_e))
        
        # Compute q2_map
        u, v = a_e[0][0], a_e[0][1]
        print("u = ", u)
        print("v = ", v)
        half_patch = int(Params.patch_size/2)
        P = s[0][:,:,u-half_patch:u+half_patch, v-half_patch:v+half_patch]
        print("P.shape = ", P.shape)
        print("H.shape = ", s[1].shape)
        q2_map = self.Q2(P, s[1], e)
        q2_a_e = q2_map[0,a_e[0][2]]
        l = Params.l * torch.ones((q2_map.size(0), q2_map.size(1)))
        l[:,a_e[0][2]] = 0
        loss_SLM2 = torch.mean(relu(q2_map + l - q2_a_e))

        return loss_SLM1 + loss_SLM2
    
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
            s, a, r, ns = self._sample(De)
            
            I = s[0]
            H = s[1]
            g = 1 if s[2] else 0

            a_e = torch.tensor(a[0:3]).unsqueeze(0)
        
            In = ns[0]
            Hn = ns[1]
            gn = 1 if ns[2] else 0

            r = r[0]

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
            
            I = s[0]
            H = s[1]
            g = 1 if s[2] else 0

            a_e = torch.tensor(a[0:3]).unsqueeze(0)
        
            In = ns[0]
            Hn = ns[1]
            gn = 1 if ns[2] else 0

            r = r[0]

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
            

        # Acquired transitions
        for i in range (Params.N/2):
            s, a, r, ns = self._sample(De)
            
            I = s[0]
            H = s[1]
            g = 1 if s[2] else 0

            a_e = torch.tensor(a[0:3]).unsqueeze(0)
        
            In = ns[0]
            Hn = ns[1]
            gn = 1 if ns[2] else 0

            r = r[0]

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
            # w = something
            L = loss_TD
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



def read_csv(path):
    with open(path, 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        data = []
        for row in csvreader:
            data.append(row)
    return data[1:]

def discretize_action(action, beta):
    '''
    beta <-- pixel dim in meters
    '''
    x,y = [float(a) for a in action[0:2]]
    quat = [float(a) for a in action[3:7]]

    L_p = 90
    L_m = beta * L_p

    # From world coords to img center coords
    x = x - 0.7
    y = y

    u = math.floor((L_m/2 - x) / beta)
    v = math.floor((L_m/2 - y) / beta)

    u = min(u,L_p - 1)
    v = min(v,L_p - 1)

    # quat to theta
    rotation = R.from_quat(np.array(quat))
    angle_axis = rotation.as_rotvec()
    theta_abs = np.linalg.norm(angle_axis)
    theta_rad = theta_abs if angle_axis[2] > 0 else (2*math.pi - theta_abs)
    theta = 2*math.pi / Params.theta_n
    j = math.floor(theta_rad / theta)
    j = min(j, Params.theta_n-1)

    assert 0 <= u and u < L_p, f"x has value {u}"
    assert 0 <= v and v < L_p, f"x has value {v}"
    assert 0 <= j and j < Params.theta_n, f"x has value {j}"

    return int(u), int(v), int(j)





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
        x, y, theta = discretize_action(action, conv_factor)
        action[0] = x
        action[1] = y
        action[2] = theta
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



