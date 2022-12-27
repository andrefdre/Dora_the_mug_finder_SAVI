#!/usr/bin/python3

import numpy as np
import torch
from PIL import Image
from torchvision import transforms


class Dataset(torch.utils.data.Dataset):
    def __init__(self,image_filenames):
        #super().init()
        self.image_filenames = image_filenames
        self.num_images= len(self.image_filenames)

        self.labels=[]

        for image_filename in self.image_filenames:
            label = self.getClassFromFilename(image_filename)
            self.labels.append(label)


        auxiliaryList = list(set(self.labels))

        print(auxiliaryList)
        # Create a set of transformations
        self.transforms = transforms.Compose([
            transforms.Resize((224,224)),
            transforms.ToTensor()
        ])
       
    def getClassFromFilename(self, filename):
        parts = filename.split('/')
        part = parts[6]
        return part


    def __getitem__(self,index): # returns a specific x,y of the datasets
        # Get the image
        image_pill = Image.open(self.image_filenames[index])

        image_t= self.transforms(image_pill)
        
        return image_t , index
       
    def __len__(self):
        return self.num_images