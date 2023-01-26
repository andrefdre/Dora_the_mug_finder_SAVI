#!/usr/bin/python3

import torch
import matplotlib.pyplot as plt
import torch.nn.functional as F
import numpy as np
from torch.autograd import Variable
from torch import nn

# Definition of the model. For now a 1 neuron network

class Model(nn.Module):
    def __init__(self):
        super().__init__()
        
        # bx3x224x224 input images
        self.layer1 = nn.Sequential(
            # 3 input channels, 16 output depth, padding and stride
            nn.Conv2d(3,32,kernel_size=5, padding=0,stride=1),
            # normalizes the batch data setting the average to 0 and std to 1
            nn.BatchNorm2d(32),
            nn.ReLU(),
            nn.MaxPool2d(2) # similar to image pyrdown, reduces size
        )

        
        self.layer2 = nn.Sequential(
            nn.Conv2d(32,64, kernel_size=5, padding=0, stride=1),
            nn.BatchNorm2d(64),
            nn.ReLU(),
            nn.MaxPool2d(2)
            )
        
        self.layer3 = nn.Sequential(
            nn.Conv2d(64,128, kernel_size=5, padding=0, stride=1),
            nn.BatchNorm2d(128),
            nn.ReLU(),
            #nn.MaxPool2d(2)
        )
        
        self.fc1 = nn.Linear(10368,512)
        self.dropout_01 = nn.Dropout(0.2)
        self.dropout_02 = nn.Dropout(0.4)
        self.dropout_03 = nn.Dropout(0.5)
        self.fc2 = nn.Linear(512,51)
        self.relu = nn.ReLU()
        self.soft_max = nn.Softmax(dim=1)
        
        
    def forward(self,x):
        # print('input x = ' + str(x.shape))
        out = self.layer1(x)
        # print('layer 1 out = ' + str(out.shape))

        out = self.layer2(out)
        # print('layer 2 out = ' + str(out.shape))

        out = self.dropout_01(out)

        out = self.layer3(out)
        # print('layer 3 out = ' + str(out.shape))

        out = self.dropout_02(out)

        out = out.view(out.size(0),-1) # flatten to keep batch dimension and compact all others into the second dimension
        # print('out after view = ' + str(out.shape))

        out = self.relu(self.fc1(out))
        # print('fc1 out = ' + str(out.shape))

        #out = out.view(out.size(0),-1) # flatten to keep batch dimension and compact all others into the second dimension
        # print('out after view = ' + str(out.shape))

        #out = self.dropout_03(out)

        out = self.fc2(out)
        # print('fc2 out = ' + str(out.shape))
        # exit(0)
        return out