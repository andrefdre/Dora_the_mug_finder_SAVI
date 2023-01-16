import random

import torch
import matplotlib.pyplot as plt
import torch.nn.functional as F
import numpy as np
from torch.autograd import Variable
from torch import nn
from torchvision import datasets, models, transforms
from utils import SaveModel,SaveGraph, LoadModel,GetClassListFromFolder


class ClassificationVisualizer():

    def __init__(self, title):
       
        # Initial parameters
        self.handles = {} # dictionary of handles per layer
        self.title = title
        self.tensor_to_pil_image = transforms.ToPILImage()
        self.class_list= GetClassListFromFolder()

    def draw(self, inputs, labels, outputs):

        # Setup figure
        self.figure = plt.figure(self.title)
        plt.axis('off')
        self.figure.canvas.manager.set_window_title(self.title)
        self.figure.set_size_inches(10,10)
        plt.suptitle(self.title)
        plt.legend(loc='best')

        inputs = inputs
        batch_size,_,_,_ = list(inputs.shape)

        output_probabilities = F.softmax(outputs, dim=1).tolist()

        random_idxs = random.sample(list(range(batch_size)), k=5*5)
        plt.clf()
        
        for plot_idx, image_idx in enumerate(random_idxs, start=1):

            output_probability=output_probabilities[image_idx]
            max_value=max(output_probability)
     
            label=output_probability.index(max(output_probability))

            if labels[image_idx].data.item() == label:
                color='green'
            else:
                color='red'

            image_t = inputs[image_idx,:,:,:]
            image_pil = self.tensor_to_pil_image(image_t)

            ax = self.figure.add_subplot(5,5,plot_idx) # define a 5 x 5 subplot matrix
            plt.imshow(image_pil)
            ax.xaxis.set_ticklabels([])
            ax.yaxis.set_ticklabels([])
            ax.xaxis.set_ticks([])
            ax.yaxis.set_ticks([])
            ax.set_xlabel(self.class_list[label], color=color)

        plt.draw()
        key = plt.waitforbuttonpress(0.05)
        if not plt.fignum_exists(1):
            print('Terminating')
            exit(0)
        