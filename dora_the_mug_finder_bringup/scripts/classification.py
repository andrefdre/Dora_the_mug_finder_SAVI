#!/usr/bin/python3

# Imports 
import argparse
import os
import glob
import random
from statistics import mean
import sys
from sklearn.model_selection import train_test_split
from torchvision import transforms
import matplotlib.pyplot as plt
import torch.nn.functional as F
from tqdm import tqdm
from colorama import Fore, Style
import torch
from PIL import Image

from dataset import Dataset
from classification_visualizer import ClassificationVisualizer
from data_visualizer import DataVisualizer
from model import Model
from utils import SaveModel,SaveGraph, LoadModel,GetClassListFromFolder




# Main code
def main():
    ########################################
    # Initialization                       #
    ########################################
    parser = argparse.ArgumentParser(description='Data Collector')
    parser.add_argument('-fn', '--folder_name', type=str, required=True, help='folder name')
    parser.add_argument('-mn', '--model_name', type=str, required=True, help='model name')
    parser.add_argument('-c', '--cuda', default=0, type=int, help='Number of cuda device')


    arglist = [x for x in sys.argv[1:] if not x.startswith('__')]
    args = vars(parser.parse_args(args=arglist))


     # General Path
    files_path=f'{os.environ["DORA"]}'
    # Image dataset paths
    image_filenames = glob.glob(files_path + '/Images SAVI Dora_the_mug_finder/*.png')


    # Define hyper parameters
    model_path = files_path + f'/models/{args["folder_name"]}/{args["model_name"]}.pkl'
    folder_path =files_path + f'/models/{args["folder_name"]}'

    class_list= GetClassListFromFolder()
    

    model = Model() # Instantiate model

    device = f'cuda:{args["cuda"]}' if torch.cuda.is_available() else 'cpu' # cuda: 0 index of gpu

    PIL_to_Tensor = transforms.Compose([
    transforms.Resize((224,224)),
    transforms.ToTensor()
    ])

    ########################################
    # Load the Model                       #
    ########################################
    model= LoadModel(model_path,model,device)
    model.eval()

    for image_filename in image_filenames:
        image_pill = Image.open(image_filename)
        
        image_t= PIL_to_Tensor(image_pill)
        image_t = image_t[0:3]
        
        image_t = image_t.unsqueeze(0)

        image_t = image_t.to(device)

        output = model(image_t)
        prediction = torch.argmax(output)
        print(class_list[prediction.data.item()])





if __name__ == '__main__':
    main()
