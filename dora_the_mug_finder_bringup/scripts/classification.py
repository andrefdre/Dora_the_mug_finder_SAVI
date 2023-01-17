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

from cv_bridge import CvBridge, CvBridgeError

import rospy
from dora_the_mug_finder_msg.msg import Object , Images


from dora_the_mug_finder_bringup.src.model import Model
from dora_the_mug_finder_bringup.src.utils import LoadModel,GetClassListFromFolder


class ImageClassifier:

    def __init__(self):
        self.pub = rospy.Publisher('class_publisher', Images, queue_size=10)
        self.images = []
        self.bridge = CvBridge()
    

    def callback(self,data):
        self.images = []
        for image in data.images:
            self.images.append(self.bridge.imgmsg_to_cv2(image, "passthrough"))


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


    ########################################
    # ROS Initialization                   #
    ########################################
    rospy.init_node('image_classifier', anonymous=False)

    image = ImageClassifier()
    rospy.Subscriber("image_publisher", Images, image.callback)

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


    while not rospy.is_shutdown():
        classification=[]
        for image_cv2 in image.images:
            image_pill = Image.fromarray(image_cv2)
            #image_pill = Image.open(image_filename)
        
            
            image_t= PIL_to_Tensor(image_pill)
            image_t = image_t[0:3]
            
            image_t = image_t.unsqueeze(0)

            image_t = image_t.to(device)

            output = model(image_t)
            prediction = torch.argmax(output)
            classification.append(class_list[prediction.data.item()])

        print(classification)
        figure = plt.figure("Visualization")  
        plt.axis('off')
        figure.figure.set_size_inches(10,10)
        plt.legend(loc='best')
        plt.clf()
        
        for plot_idx, image_idx in enumerate(list(range(len(image.images))), start=1):
        
            label=classification[image_idx]

            image_pil = Image.fromarray(image.images[image_idx])

            ax = figure.figure.add_subplot(5,5,plot_idx) # define a 5 x 5 subplot matrix
            plt.imshow(image_pil)
            ax.xaxis.set_ticklabels([])
            ax.yaxis.set_ticklabels([])
            ax.xaxis.set_ticks([])
            ax.yaxis.set_ticks([])
            ax.set_xlabel(classification[image_idx])

        plt.draw()
        key = plt.waitforbuttonpress(0)
        if not plt.fignum_exists(1):
            print('Terminating')
            exit(0)

        rospy.sleep(1) 




if __name__ == '__main__':
    main()
