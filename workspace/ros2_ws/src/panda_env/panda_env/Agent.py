import torch
import torch.nn.functional as F

class Agent:
    def __init__(self):
        # Initialize any necessary attributes here
        self.Q1 = None
        self.Q2 = None
        self.Q1_target = None
        self.Q2_tsrget = None
        pass

    def l(self, a: torch.Tensor, a_e: torch.Tensor) -> torch.Tensor:
        '''the non expert action penalty
        a: tensor with actions of the agent
        a_e: tensor with actions of the expert
        
        return l: the loss of the non expert
        '''
        # large margin loss penalty for non-expert actions
        # https://arxiv.org/pdf/1704.03732
        l = F.relu(a - a_e + margin)
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

        #Q1 does not use a, so it's not needed
        Q1_pred = self.Q1(s[0], s[1], s[2])[:, a1]
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

        Q2_pred = self.Q2(s[0], s[1], s[2], e,  a[:,0:2])[:, a[:,2]]
        loss = F.huber_loss(Q2_pred, y)
        return loss

    def lossSLM(self, s: tuple, a: torch.Tensor, a_e: torch.Tensor) -> torch.Tensor:
        '''calculate the SLM loss
        s: it's a tuple of 3 dimension containing 3 tensor, one for the 
            height_images, one for the in hand images and one for the
            boolean related to the gripper status
            
        a: tensor of actions
        a_e: tensor of expert actions
        y: TD target
        
        return the TD loss for Q2
        '''
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

    def train(De: torch.Torch):
        #Pre_training for M
        gamma = 1
        s = De[0]
        sn = De[2]
        a_e = De[1]
        r = De[4]
        #a1 are the x,y coordinates
        a1_n = torch.argmax(self.Q1_target(sn[0], sn[1], sn[2])[:, 1])
        #a2 is the rotation angle
        e = self.Q1_target(sn[0], sn[1], sn[2])[:, 0]
        a2_n = torch.argmax(self.Q2_target(sn[0], sn[1], sn[2], e, a1_n))
        y = r + gamma*torch.max(a2)

        a1 = torch.argmax(self.Q1(s[0], s[1], s[2])[:, 1])
        e = self.Q1(s[0], s[1], s[2])[:, 0]
        a2 = torch.argmax(self.Q2(sn[0], s[1], s[2], e, a2))

        ltd_loss = self.lossTD1(s, a1, y) + lossTD2(s, torch.concatenate(a1, a2), e, y)
        lslm = self.LSLM(s,torch.concatenate(a1, a2), a_e)
        # w = something
        L = ltd_loss + w* lslm
        # gradient descent

        # Training for N:
        # Copy some part before and self play
        #  better understand how to talk with the env





