import torch
import csv
from models import ResUCatShared, InHandConv, CNNShared
import copy
import matplotlib.pyplot as plt
from PIL import Image
from torchvision import transforms
import os
import torch.nn.functional as F


class Params:
    patch_size = 24
    theta_resolution = 180
    l = 1
    margin = 0
    gamma = 0.99
    w = 0.5

class Agent:
    def __init__(self):
        self.q1 = ResUCatShared()
        self.q1_target = copy.deepcopy(self.q1)
        self.q2 = CNNShared((2, Params.patch_size, Params.patch_size),Params.theta_resolution)
        self.q2_target = copy.deepcopy(self.q2)

    def Q1(self, I, H, g):
        q1_map, e = self.q1(I, H)
        return q1_map[:,g,:,:], e
    
    def Q1_traget(self, I, H, g):
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
        l = Params.l * F.relu(a - a_e + Params.margin)
        return l

    def lossTD1(self, s: tuple, a: torch.Tensor, y: torch.Tensor) -> torch.Tensor:
        '''calculate the first TD loss for Q1
        s: it's a tuple of 3 dimension containing 3 tensor, one for the 
            height_images, one for the in hand images and one for the
            boolean related to the gripper status
            
        a: tensor of actions
        y: TD target
        
        return the TD loss for Q1
        '''
        Q1_pred = self.Q1(s[0], s[1], s[2])[:, a]
        loss = F.huber_loss(Q1_pred, y)
        return loss

    def lossTD2(self, s: tuple, a: torch.Tensor, e: torch.Tensor, y: torch.Tensor) -> torch.Tensor:
        '''calculate the first TD loss for Q2
        s: it's a tuple of 3 dimension containing 3 tensor, one for the 
            height_images, one for the in hand images and one for the
            boolean related to the gripper status
        
        e: tensor for encoding
            
        a: tensor of actions
        y: TD target
        
        return the TD loss for Q2
        '''

        Q2_pred = self.Q2(s[0], s[1], s[2], e)[:, a]
        loss = F.huber_loss(Q2_pred, y)
        return loss

    def lossSLM(self, s: tuple, a_e: torch.Tensor) -> torch.Tensor:
        '''calculate the SLM loss
        s: it's a tuple of 3 dimension containing 3 tensor, one for the 
            height_images, one for the in hand images and one for the
            boolean related to the gripper status
            
        a: tensor of actions
        a_e: tensor of expert actions
        y: TD target
        
        return the TD loss for Q2
        '''

        # TODO: recheck tomorrow, we don't need the action a

        # Calculating the strict large margin loss (SLM)
        margin_loss = self.l(a, a_e)
        e = self.Q1(s[0], s[1], s[2])[:, 0]
        Q_s_a1 = self.Q1(s[0], s[1], s[2])[:, 1] 
        #I take the Q values at the expert position
        Q_s_a_e1 = self.Q1(s[0], s[1], s[2])[a_e[:,0:2]][:, 1]
        a1 = torch.argmax(Q_s_a1)
        Q_s_a = self.Q2(s[0], s[1], s[2], e, a1)  # Assuming Q is a network attribute
        Q_s_a_e = self.Q2(s[0], s[1], s[2], e, a_e[:,0:2])
        index = Q_s_a[Q_s_a > Q_s_a_e - margin_loss]
        LSLM = torch.mean(Q_s_a[index] + self.l(a, a_e)[index] - Q_s_a_e[index])
        return LSLM

    def train(self, De: torch.Torch):
        #Pre_training for M
        s = De[0]
        sn = De[2]
        a_e = De[1]
        r = De[4]
        #a1 are the x,y coordinates
        a1_n = torch.argmax(self.Q1_target(sn[0], sn[1], sn[2])[:, 1])
        #a2 is the rotation angle
        e = self.Q1_target(sn[0], sn[1], sn[2])[:, 0]
        a2_n = torch.argmax(self.Q2_target(sn[0], sn[1], sn[2], e, a1_n))
        y = r + Params.gamma*torch.max(a2)

        a1 = torch.argmax(self.Q1(s[0], s[1], s[2])[:, 1])
        e = self.Q1(s[0], s[1], s[2])[:, 0]
        a2 = torch.argmax(self.Q2(sn[0], s[1], s[2], e, a2))

        ltd_loss = self.lossTD1(s, a1, y) + self.lossTD2(s, torch.concatenate(a1, a2), e, y)
        lslm = self.LSLM(s,torch.concatenate(a1, a2), a_e)
        # w = something
        L = ltd_loss + Params.w* lslm
        # gradient descent

        # Training for N:
        # Copy some part before and self play
        #  better understand how to talk with the env


def read_csv(path):
    with open(path, 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        data = []
        for row in csvreader:
            data.append(row)
    return data[1:]

if __name__ == "__main__":
    agent = Agent()
    path = 'dataset/trajectory_1'

    states_path = os.path.join(path,"states.csv")
    actions_path = os.path.join(path,"actions.csv")
    rewards_path = os.path.join(path,"rewards.csv")
    next_states_path = os.path.join(path,"next_states.csv")

    states = read_csv(states_path)
    actions = read_csv(actions_path)
    rewards = read_csv(rewards_path)
    next_states = read_csv(next_states_path)

    path_height_map = os.path.join(path, "imgs", states[0][0])
    path_in_hand_img = os.path.join(path, "imgs", states[0][1])

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

    a = agent.predict(height_map, in_hand_img, 0)

    print("a = ", a)
