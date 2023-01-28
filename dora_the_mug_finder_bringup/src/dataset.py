#!/usr/bin/python3

import numpy as np
import torch
from PIL import Image
from torchvision import transforms
import glob
import os
from dora_the_mug_finder_bringup.src.utils import GetClassListFromFolder

class Dataset(torch.utils.data.Dataset):
    def __init__(self,image_filenames):
        #super().init()
        self.image_filenames = image_filenames
        self.num_images= len(self.image_filenames)

        self.labels_string=[]

        for image_filename in self.image_filenames:
            label = self.getClassFromFilename(image_filename)
            self.labels_string.append(label)

        # Gets the list of the items
        self.classes_list = GetClassListFromFolder()

        # Creates the labels comparing the string of the item with the folder structure
        self.labels=[]
        for label in self.labels_string:
            label_idx=self.classes_list.index(label)
            self.labels.append(label_idx)

        # Create a set of transformations
        self.transforms = transforms.Compose([
            #transforms.Resize((64,64)),
            transforms.ToTensor()
        ])
    
    # Function that will get the class from the file structure
    def getClassFromFilename(self, filename):
        parts = filename.split('/')
        # Get the third item counting from the end
        part = parts[len(parts)-3]
        return part

    def expand2square(self,img_array):
        height, width , _ = img_array.shape
        if width == height == 64:
            return img_array
        elif width<64 and height<64:
            img_pill = np.zeros((64,64,3), np.uint8)
            if height%2==0 and width%2==0:
                img_pill[32-int(height/2):32+int(height/2),32-int(width/2):32+int(width/2)]=img_array
            elif height%2==0 and width%2!=0:
                img_pill[32-int(height/2):32+int(height/2),32-int(width/2):32+int(width/2)+1]=img_array
            elif height%2!=0 and width%2==0:
                img_pill[32-int(height/2):32+int(height/2)+1,32-int(width/2):32+int(width/2)]=img_array
            else:
                img_pill[32-int(height/2):32+int(height/2)+1,32-int(width/2):32+int(width/2)+1]=img_array

            for i in range(0,32-int(height/2)):
                img_pill[i,:,:]=img_pill[32-int(height/2)+1,:,:]
            for i in range(32+int(height/2),64):
                img_pill[i,:,:]=img_pill[32+int(height/2)-1,:,:]

            for i in range(0,32-int(width/2)):
                img_pill[:,i,:]=img_pill[:,32-int(width/2)+1,:]
            for i in range(32+int(width/2),64):
                img_pill[:,i,:]=img_pill[:,32+int(width/2)-1,:]
            
            return img_pill
        elif width > height:
            img_pill = np.zeros((64,64,3), np.uint8)
            if height%2==0:
                img_pill[32-int(height/2):32+int(height/2),0:width]=img_array
            else:
                img_pill[32-int(height/2):32+int(height/2)+1,0:width]=img_array
            
            for i in range(0,32-int(height/2)):
                img_pill[i,:,:]=img_pill[32-int(height/2)+1,:,:]
            for i in range(32+int(height/2),64):
                img_pill[i,:,:]=img_pill[32+int(height/2)-1,:,:]

            return img_pill
        else:
            img_pill = np.zeros((64,64,3), np.uint8)

            if width%2==0:
                img_pill[0:height,32-int(width/2):32+int(width/2)]=img_array
            else:
                img_pill[0:height,32-int(width/2):32+int(width/2)+1]=img_array
            
            for i in range(0,32-int(width/2)):
                img_pill[:,i,:]=img_pill[:,32-int(width/2)+1,:]
            for i in range(32+int(width/2),64):
                img_pill[:,i,:]=img_pill[:,32+int(width/2)-1,:]

            return img_pill


    def __getitem__(self,index): # returns a specific x,y of the datasets
        # Get the image
        image_pill = Image.open(self.image_filenames[index])

        image_pill.thumbnail((64,64))

        image_array = np.asarray(image_pill)

        image_padded=self.expand2square(image_array)

        #image_padded.setflags(write=1)

        image_t= self.transforms(np.array(image_padded))

        label = self.labels[index]
        
        return image_t , label
       
    def __len__(self):
        return self.num_images