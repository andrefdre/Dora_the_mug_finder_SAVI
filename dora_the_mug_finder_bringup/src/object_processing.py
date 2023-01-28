import numpy as np
import open3d as o3d
import cv2

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


def get_object_color(mean_color):
    
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

def get_object_dimension(points, multiplier=1):

    # get the bounding box    
    bbox_max = points.get_max_bound()
    bbox_min = points.get_min_bound()
    
    # get the dimension
    height = abs(abs(bbox_max[1])-abs(bbox_min[1]))*multiplier 
    length = abs(abs(bbox_max[0])-abs(bbox_min[0]))*multiplier 
    width = abs(abs(bbox_max[2])-abs(bbox_min[2]))*multiplier 

    # round the dimension
    if multiplier != 1:
        height = round(height)
        length = round(length)
        width = round(width)

    return height, width, length

