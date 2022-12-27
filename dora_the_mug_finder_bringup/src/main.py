#!/usr/bin/python3

from dataset import Dataset
import os
import glob
import random
from sklearn.model_selection import train_test_split
from torchvision import transforms
import matplotlib.pyplot as plt
from tqdm import tqdm
from colorama import Fore, Style
import torch


def main():
    dataset_path=f'{os.environ["DORA"]}/rgbd-dataset'
    image_filenames = glob.glob(dataset_path + '/*/*/*_crop.png')

    tensor_to_pill_image = transforms.Compose([
        transforms.ToPILImage()
    ])

    # Sample ony a few images for develop
    image_filenames = random.sample(image_filenames,k=700)
    train_image_filenames,test_image_filenames = train_test_split(image_filenames,test_size=0.2)

    # Creates the train dataset
    dataset_train = Dataset(train_image_filenames)
    # Creates the batch size that suits the amount of memory the graphics can handle
    loader_train = torch.utils.data.DataLoader(dataset=dataset_train,batch_size=256,shuffle=True)
    # Goes through al the images and displays them
    for image_t ,label_t in loader_train:
        num_images = image_t.shape[0]
        image_idxs=random.sample(range(0,num_images),k=25)

        fig = plt.figure()
        fig.subplots_adjust(hspace=0.5)
        for plot_idx,image_idx in enumerate(image_idxs,start=1):
            image = tensor_to_pill_image(image_t[image_idx,:,:,:]) # get images idx image_idx
            ax = fig.add_subplot(5,5,plot_idx) ## Create subplot
            ax.xaxis.set_ticklabels([])
            ax.yaxis.set_ticklabels([])
            ax.xaxis.set_ticks([])
            ax.yaxis.set_ticks([])
            label =label_t[image_idx]
            ax.set_xlabel(label)
            plt.imshow(image)

        plt.show()

if __name__ == '__main__':
    main()
