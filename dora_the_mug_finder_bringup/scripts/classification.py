#!/usr/bin/python3

# General Imports 
import argparse
import os
import glob
import sys
from torchvision import transforms
import matplotlib.pyplot as plt
from colorama import Fore, Style
import torch
from PIL import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy
from std_msgs.msg import String

# Own package imports
from dora_the_mug_finder_msg.msg import Object , Images , Classes
from dora_the_mug_finder_bringup.src.model import Model
from dora_the_mug_finder_bringup.src.utils import LoadModel,GetClassListFromFolder


class ImageClassifier:

    def __init__(self,model,device):
        self.model = model
        self.device = device
        self.pub = rospy.Publisher('class_publisher', Classes, queue_size=10)
        self.images = []
        self.bridge = CvBridge()
        self.class_list= GetClassListFromFolder()
        self.PIL_to_Tensor = transforms.Compose([
                            transforms.Resize((224,224)),
                            transforms.ToTensor()
                            ])

        self.figure = plt.figure("Visualization")  
        plt.axis('off')
        self.figure.figure.set_size_inches(10,4)
        plt.legend(loc='best')
        plt.clf()
        pub = rospy.Publisher('objects_publisher', Object, queue_size=10)
        self.update_graph = False
                            

    def callback(self,data):
        self.images = []
        for image in data.images:
            self.images.append(self.bridge.imgmsg_to_cv2(image, "passthrough"))

        self.classification=Classes()
        for image_cv2 in self.images:
            image_pill = Image.fromarray(image_cv2)
            #image_pill = Image.open(image_filename)
        
            
            image_t= self.PIL_to_Tensor(image_pill)
            image_t = image_t[0:3]
            
            image_t = image_t.unsqueeze(0)

            image_t = image_t.to(self.device)

            output = self.model(image_t)
            prediction = torch.argmax(output)
            self.classification.classes.append(String(self.class_list[prediction.data.item()]))
        
        self.pub.publish(self.classification)
        self.update_graph = True

        print(self.classification)
        

    def draw(self):
        plt.clf()
        for plot_idx, image_idx in enumerate(list(range(len(self.images))), start=1):
        
            label=self.classification.classes[image_idx].data

            image_pil = Image.fromarray(self.images[image_idx])

            ax = self.figure.figure.add_subplot(2,5,plot_idx) # define a 5 x 5 subplot matrix
            plt.imshow(image_pil)
            ax.xaxis.set_ticklabels([])
            ax.yaxis.set_ticklabels([])
            ax.xaxis.set_ticks([])
            ax.yaxis.set_ticks([])
            ax.set_xlabel(self.classification.classes[image_idx].data)

        self.update_graph = False
        plt.draw()
        key = plt.waitforbuttonpress(1)
        if not plt.fignum_exists(1):
            print('Terminating')
            exit(0)


# Main code
def main():
    ########################################
    # Initialization                       #
    ########################################
    parser = argparse.ArgumentParser(description='Data Collector')
    parser.add_argument('-fn', '--folder_name', type=str, required=True, help='folder name')
    parser.add_argument('-mn', '--model_name', type=str, required=True, help='model name')
    parser.add_argument('-c', '--cuda', default=0, type=int, help='Number of cuda device')
    parser.add_argument('-v', '--visualize', action='store_true', help='Visualize the results')


    arglist = [x for x in sys.argv[1:] if not x.startswith('__')]
    args = vars(parser.parse_args(args=arglist))

    device = f'cuda:{args["cuda"]}' if torch.cuda.is_available() else 'cpu' # cuda: 0 index of gpu

    model = Model() # Instantiate model

    # General Path
    files_path=f'{os.environ["DORA"]}'
    # Image dataset paths
    image_filenames = glob.glob(files_path + '/Images SAVI Dora_the_mug_finder/*.png')


    # Define hyper parameters
    model_path = files_path + f'/models/{args["folder_name"]}/{args["model_name"]}.pkl'
    folder_path =files_path + f'/models/{args["folder_name"]}'

    ########################################
    # Load the Model                       #
    ########################################
    model= LoadModel(model_path,model,device)
    model.eval()


    ########################################
    # ROS Initialization                   #
    ########################################
    rospy.init_node('image_classifier', anonymous=False)

    image = ImageClassifier(model,device)

    rospy.Subscriber("image_publisher", Images, image.callback)

    rate = rospy.Rate(10) # 10hz
    


    while not rospy.is_shutdown():
        if args['visualize'] and image.update_graph==True: # Checks if the user wants to visualize the point cloud
            image.draw()
        rate.sleep() # Sleeps to if time < rate




if __name__ == '__main__':
    main()
