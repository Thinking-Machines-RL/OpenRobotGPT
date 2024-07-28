import torch
import pandas as pd
from models import ResUCatShared, InHandConv, CNNShared
import copy
import matplotlib.pyplot as plt
from PIL import Image
from torchvision import transforms

class Agent:
    def __init__(self):
        self.q1 = ResUCatShared()
        self.q1_target = copy.deepcopy(self.q1)
        self.q2 = {"InHandConv" : InHandConv(), "CNNShared" : CNNShared()}
        self.q2_target = (copy.deepcopy(self.q2[0]), copy.deepcopy(self.q2[1]))

    def Q1(self, I, H, g):
        q1_map, e = self.q1(I, H)
        return q1_map[g], e
    
    def Q1_traget(self, I, H, g):
        q1_map, e = self.q1_target(I, H)
        return q1_map[g], e

    def Q2(self, P, H, e):
        patch = torch.cat((P,H),1)  # one image next to the other
        processed_patch = self.q2["InHandConv"](patch)
        q2_map = self.q2["CNNShared"](processed_patch, e)
        return q2_map
    
    def Q2_target(self, P, H, e):
        patch = torch.cat((P,H),1)  # one image next to the other
        processed_patch = self.q2_target["InHandConv"](patch)
        q2_map = self.q2_target["CNNShared"](processed_patch, e)
        return q2_map

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

    def predict(self, I, H, g):
        q1_map, e = self.Q1(I, H, g)

        # Find optimal (u,v)
        uv = torch.argmax(q1_map)
        u, v = uv[0], uv[1]

        # Crop patch
        PATCH_SIZE = 24
        padded_size = I.shape[0] + PATCH_SIZE
        padded_img = torch.zeros((padded_size, padded_size))
        padded_img[PATCH_SIZE/2:padded_size-PATCH_SIZE/2, PATCH_SIZE/2:padded_size-PATCH_SIZE/2] = I

        P = padded_img[u-PATCH_SIZE/2:u+PATCH_SIZE/2,v-PATCH_SIZE/2:v+PATCH_SIZE/2]

        q2_map = self.Q2(P, H, e)

        # Find optimal theta
        j = torch.argmax(q2_map)
        return u,v,j

if __name__ == "__main__":
    agent = Agent()
    path_height_map = "height_map_0.png"
    path_in_hand_img = "in_hand_img_0.png"

    height_map = Image.open(path_height_map)
    in_hand_img = Image.open(path_in_hand_img)

    plt.imshow(height_map)
    plt.show()
    plt.imshow(in_hand_img)
    plt.show()

    height_map_t = transforms(height_map)
    in_hand_img_t = transforms(in_hand_img)

    a = agent.predict(height_map_t, in_hand_img_t, 0)

    print("a = ", a)
