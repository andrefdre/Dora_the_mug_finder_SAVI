import torch
import matplotlib.pyplot as plt
import open3d as o3d
import numpy as np
import os
import glob


def SaveModel(model,idx_epoch,optimizer,epoch_train_losses,epoch_test_losses,model_path,device):
    model.to('cpu')
    torch.save({
        'epoch': idx_epoch,
        'model_state_dict': model.state_dict(),
        'optimizer_state_dict': optimizer.state_dict(),
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

def text_3d(text, font='/usr/share/fonts/truetype/freefont/FreeMono.ttf', font_size=10):
    """
    Generate a 3D text point cloud used for visualization.
    :param text: content of the text
    :param font: Name of the font - change it according to your system
    :param font_size: size of the font
    :return: o3d.geoemtry.PointCloud object
    """

    from PIL import Image, ImageFont, ImageDraw

    font_obj = ImageFont.truetype(font, font_size)
    font_dim = font_obj.getsize(text)

    img = Image.new('RGB', font_dim, color=(255, 255, 255))
    draw = ImageDraw.Draw(img)
    draw.text((0, 0), text, font=font_obj, fill=(0, 0, 0))
    img = np.asarray(img)
    img_mask = img[:, :, 0] < 128
    indices = np.indices([*img.shape[0:2], 1])[:, img_mask, 0].reshape(3, -1).T

    pcd = o3d.geometry.PointCloud()
    pcd.colors = o3d.utility.Vector3dVector(img[img_mask, :].astype(float) / 255.0)
    pcd.points = o3d.utility.Vector3dVector(indices / 100.0)

    return pcd