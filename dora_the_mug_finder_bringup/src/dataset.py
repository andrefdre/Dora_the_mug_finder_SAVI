#!/usr/bin/python3

import numpy as np
import torch
from PIL import Image
from torchvision import transforms
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
        half_height=int(height/2)
        half_width=int(width/2)
        # If the image is already 64x64, return it
        if width == height == 64:
            return img_array
        # If the image is bigger than 64x64, crop it and pad it
        # If width is smaller than 64 and height is smaller than 64, pad the height and width
        elif width<64 and height<64:    # Check if image width and height is less than 64
            img_pill = np.zeros((64,64,3), np.uint8)    # Create a 64x64x3 size of zero array
            if height%2==0 and width%2==0:    # check both height and width are even or not
                img_pill[32-half_height:32+half_height,32-half_width:32+half_width]=img_array    # assign the image array in the centre of 64x64 image
            elif height%2==0 and width%2!=0:    # check if height is even and width is odd
                img_pill[32-half_height:32+half_height,32-half_width:32+half_width+1]=img_array    # assign the image array in the centre of 64x64 image
            elif height%2!=0 and width%2==0:    # check if height is odd and width is even
                img_pill[32-half_height:32+half_height+1,32-half_width:32+half_width]=img_array    # assign the image array in the centre of 64x64 image
            else:    # else body
                img_pill[32-half_height:32+half_height+1,32-half_width:32+half_width+1]=img_array    # assign the image array in the centre of 64x64 image

            for i in range(0,32-half_height):    # Loop from 0 to 32-half height
                img_pill[i,:,:]=img_pill[32-half_height,:,:]    # assign the value of img_pill[32-half_height,:,:]
            if height%2==0:    # check if height is even
                for i in range(32+half_height,64):    # loop from 32+half_height to 64
                    img_pill[i,:,:]=img_pill[32+half_height-1,:,:]    # assign the value of img_pill[32+half_height-1,:,:]
            else:    # else body
                for i in range(32+half_height,64):    # loop from 32+half_height to 64
                    img_pill[i,:,:]=img_pill[32+half_height,:,:]    # assign the value of img_pill[32+half_height,:,:]

            for i in range(0,32-half_width):    # Loop from 0 to 32-half width
                img_pill[:,i,:]=img_pill[:,32-half_width,:]    # assign the value of img_pill[:,32-half_width,:]
            if width%2==0:    # check if width is even
                for i in range(32+half_width,64):    # loop from 32+half_width to 64
                    img_pill[:,i,:]=img_pill[:,32+half_width-1,:]    # assign the value of img_pill[:,32+half_width-1,:]
            else:    # else body
                for i in range(32+half_width,64):    # loop from 32+half_width to 64
                    img_pill[:,i,:]=img_pill[:,32+half_width,:]    # assign the value of img_pill[:,32+half_width,:]
            
            return img_pill    # return the image of 64x64x3 size

        elif width > height:    # check if width is greater than height
            img_pill = np.zeros((64,64,3), np.uint8)    # initialize an array with zeros with 64x64x3 dimensions
            if height%2==0:    # check if height is even or odd
                img_pill[32-half_height:32+half_height,0:width]=img_array    # copy the image to center of new image
            else:    # else body
                img_pill[32-half_height:32+half_height+1,0:width]=img_array    # copy the image to center of new image
            
            for i in range(0,32-half_height):    # Loop through the first half of image
                img_pill[i,:,:]=img_pill[32-half_height,:,:]    # Copy the first row of the image

            if height%2==0:    # check if height is even or odd
                for i in range(32+half_height,64):    # Loop through the second half of image
                    img_pill[i,:,:]=img_pill[32+half_height-1,:,:]    # Copy the last row of the image
            else:    # else body
                for i in range(32+half_height,64):    # Loop through the second half of image
                    img_pill[i,:,:]=img_pill[32+half_height,:,:]    # Copy the last row of the image
            return img_pill    # return img_pill

        else:
            img_pill = np.zeros((64,64,3), np.uint8)    # create an image of size 64x64 with 3 channels
            if width%2==0:    # if width of image is even
                img_pill[0:height,32-half_width:32+half_width]=img_array    # copy the image to center of new image
            else:    # else body
                img_pill[0:height,32-half_width:32+half_width+1]=img_array    # copy the image to center of new image
            
            for i in range(0,32-half_width):    # loop from 0 to half width of the image
                img_pill[:,i,:]=img_pill[:,32-half_width,:]    # copy the left most column to the left of new image
            
            if width%2==0:    # if width of image is even
                for i in range(32+half_width,64):    # loop from right end of image to right end of new image
                    img_pill[:,i,:]=img_pill[:,32+half_width-1,:]    # copy the right most column to the right of new image
            else:    # else body
                for i in range(32+half_width,64):    # loop from right end of image to right end of new image
                    img_pill[:,i,:]=img_pill[:,32+half_width,:]    # copy the right most column to the right of new image
            return img_pill    # return the new image


    def __getitem__(self,index): # returns a specific x,y of the datasets
        # Get the image
        image_pill = Image.open(self.image_filenames[index])
        # Resize the image without changing the aspect ratio
        image_pill.thumbnail((64,64))
        # Convert the image to a numpy array
        image_array = np.asarray(image_pill)
        # Pad the image to 64x64
        image_padded=self.expand2square(image_array)
        # Convert the image to a tensor
        image_t= self.transforms(np.array(image_padded))
        # Get the label
        label = self.labels[index]
        return image_t , label
       
    def __len__(self):
        return self.num_images