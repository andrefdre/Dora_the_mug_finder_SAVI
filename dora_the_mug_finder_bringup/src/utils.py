import torch
import matplotlib.pyplot as plt
import open3d as o3d
import numpy as np
import cv2
import os
import glob


def SaveModel(model,idx_epoch,optimizer,training_loader,testing_loader,epoch_train_losses,epoch_test_losses,model_path,device):
    model.to('cpu')
    torch.save({
        'epoch': idx_epoch,
        'model_state_dict': model.state_dict(),
        'optimizer_state_dict': optimizer.state_dict(),
        'loader_train': training_loader,
        'loader_test': testing_loader,
        'train_losses': epoch_train_losses,
        'test_losses': epoch_test_losses,
        }, model_path)
    model.to(device)

def LoadModel(model_path,model,device):
    checkpoint = torch.load(model_path)
    model.load_state_dict(checkpoint['model_state_dict'])
    model.to(device) # move the model variable to the gpu if one exists
    return model

def SaveGraph(train_losses,test_losses,folder_name):
    plt.figure()
    plt.plot(train_losses, label='train loss')
    plt.plot(test_losses, label='test loss')
    plt.xlabel("Epoch")    
    plt.ylabel("Loss")   
    plt.legend()
    plt.savefig(f'{folder_name}/losses.png')

def GetClassListFromFolder():
    dataset_path=f'{os.environ["DORA"]}/rgbd-dataset'
    folder_names = glob.glob(dataset_path + '/*')
    classList=[]
    for folder_name in folder_names:
        parts = folder_name.split('/')
        part = parts[-1]
        classList.append(part)
    return classList

def text_3d(text,density=10,font='/usr/share/fonts/truetype/freefont/FreeMono.ttf', font_size=10):
    """
    Generate a 3D text point cloud used for visualization.
    :param text: content of the text
    :param font: Name of the font - change it according to your system
    :param font_size: size of the font
    :return: o3d.geoemtry.PointCloud object
    """

    from PIL import Image, ImageFont, ImageDraw

    font_obj = ImageFont.truetype(font, font_size*density)
    font_dim = font_obj.getsize(text)

    img = Image.new('RGB', font_dim, color=(255, 255, 255))
    draw = ImageDraw.Draw(img)
    draw.text((0, 0), text, font=font_obj, fill=(0, 0, 0))
    img = np.asarray(img)
    img_mask = img[:, :, 0] < 128
    indices = np.indices([*img.shape[0:2], 1])[:, img_mask, 0].reshape(3, -1).T
    
    pcd = o3d.geometry.PointCloud()
    pcd.colors = o3d.utility.Vector3dVector(img[img_mask, :].astype(float) / 255.0)
    pcd.points = o3d.utility.Vector3dVector(indices / 100.0 / density)

    return pcd


def get_color(mean_color):
    
    # convert to rgb
    mean_color = mean_color*255
    rgb_value= np.array([[mean_color]], dtype=np.uint8)

    # convert to hsv
    hsv = cv2.cvtColor(rgb_value, cv2.COLOR_RGB2HSV)
    
    # get the hsv value
    H,S,V = cv2.split(hsv)
   
    # get the color
    if H < 10 and S > 50:
        color = 'red'
    elif H < 15 and S > 50:
        color = 'orange'
    elif H < 30 and S > 50:
        color = 'yellow'
    elif H < 75 and S > 50:
        color = 'green'
    elif H < 125 and S > 50:
        color = 'blue'
    elif H < 155 and S > 50:
        color = 'purple'
    elif H < 180 and S > 50:
        color = 'red'
    elif S <= 50 and V < 90:
        color = 'black'
    else:
        color = 'white'

    return color