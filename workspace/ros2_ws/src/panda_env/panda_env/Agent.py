import torch
import csv
from models import ResUCatShared, InHandConv, CNNShared
import copy
import matplotlib.pyplot as plt
from PIL import Image
from torchvision import transforms


class Params:
    patch_size = 24
    theta_resolution = 180

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

    def l(a,a_e):
        pass

    def lossSLM(s,a_e):
        pass

    def lossTD1(s1,a1):
        pass

    def lossTD2(s2,a2):
        pass

    def train(self, dataset):
        pass

if __name__ == "__main__":
    agent = Agent()
    path_height_map = "height_map_0.png"
    path_in_hand_img = "in_hand_img_0.png"

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
