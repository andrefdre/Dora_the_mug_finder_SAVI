#!/usr/bin/env python3

# General Imports 
import PySimpleGUI as sg
import rospy
import os
import PIL.Image as Image
import glob
import io
import rospkg


from std_msgs.msg import String


def convToBytes(image, resize=None):
	img = image.copy()	
	cur_width, cur_height = img.size
	if resize:
		new_width, new_height = resize
		scale = min(new_height/cur_height, new_width/cur_width)
		img = img.resize((int(cur_width*scale), int(cur_height*scale)), Image.Resampling.LANCZOS)	
	ImgBytes = io.BytesIO()
	img.save(ImgBytes, format="PNG")
	del img
	return ImgBytes.getvalue()


# Main code
def main():
    #################################
    # Initialization                #
    #################################
    files_path=f'{os.environ["DORA"]}'

    filenames = glob.glob(files_path + '/rgbd-scenes-v2/imgs/*/00000-color.png')
    
    rospack = rospkg.RosPack()
    path = rospack.get_path('dora_the_mug_finder_bringup')
    parts = path.split('/')
    logo_path =''
    kinect_path =''
    for part in parts[0:len(parts)-1]:
        logo_path += part + '/'
        kinect_path += part + '/'

    logo_path += '/Docs/logo_png.png'
    kinect_path += '/Docs/kinect.png'


    #################################
    # User interface initialization #
    #################################
    first_col = [[sg.Button("scene_01")], [sg.Image(convToBytes(Image.open(filenames[0]),resize=(100,100)))] ,[sg.Button("scene_02")], [sg.Image(convToBytes(Image.open(filenames[1]),resize=(100,100)))] , [sg.Button("scene_03")] , [sg.Image(convToBytes(Image.open(filenames[2]),resize=(100,100)))]]
    second_col = [[sg.Button("scene_04")] , [sg.Image(convToBytes(Image.open(filenames[3]),resize=(100,100)))] , [sg.Button("scene_05")] , [sg.Image(convToBytes(Image.open(filenames[4]),resize=(100,100)))] , [sg.Button("scene_06")], [sg.Image(convToBytes(Image.open(filenames[5]),resize=(100,100)))]]
    third_col = [[sg.Button("scene_07")] , [sg.Image(convToBytes(Image.open(filenames[6]),resize=(100,100)))] , [sg.Button("scene_08")] , [sg.Image(convToBytes(Image.open(filenames[7]),resize=(100,100)))] , [sg.Button("scene_09")] , [sg.Image(convToBytes(Image.open(filenames[8]),resize=(100,100)))]]
    fourth_col = [[sg.Button("scene_10")] , [sg.Image(convToBytes(Image.open(filenames[9]),resize=(100,100)))] , [sg.Button("scene_11")] , [sg.Image(convToBytes(Image.open(filenames[10]),resize=(100,100)))] , [sg.Button("scene_12")] , [sg.Image(convToBytes(Image.open(filenames[11]),resize=(100,100)))]]
    fifth_col = [[sg.Button("scene_13")] , [sg.Image(convToBytes(Image.open(filenames[12]),resize=(100,100)))] , [sg.Button("scene_14")] , [sg.Image(convToBytes(Image.open(filenames[13]),resize=(100,100)))] , [sg.Button("Kinect")] , [sg.Image(convToBytes(Image.open(kinect_path),resize=(100,100)))] ]
    logo_col = [[sg.Text("Dora The Mug Finder")] , [sg.Image(convToBytes(Image.open(logo_path),resize=(300,300)))]]
    layout = [[sg.Column(logo_col, element_justification='c'),sg.VSeperator(color='#505050'),sg.VSeperator(color='#505050') , sg.Column(first_col, element_justification='c'),sg.VSeperator(color='#505050'),sg.Column(second_col, element_justification='c'),sg.VSeperator(color='#505050'),sg.Column(third_col, element_justification='c'),
               sg.VSeperator(color='#505050'),sg.Column(fourth_col, element_justification='c') , sg.VSeperator(color='#505050') , sg.Column(fifth_col, element_justification='c')]]

    # Create the window
    window = sg.Window("Dora The Mug Finder", layout,resizable=True)

    #################################

    # ROS Initialization            #
    #################################
    rospy.init_node('user_interface', anonymous=False)
    pub = rospy.Publisher('scene_publisher', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    # Create an event loop
    while not rospy.is_shutdown():
        event, values = window.read()
        # End program if user closes window or
        # presses the OK button
        if event == sg.WINDOW_CLOSED:
            break

        if event == "scene_01":
            pub.publish(String("scene_01"))
        elif event == "scene_02":
            pub.publish(String("scene_02"))
        elif event == "scene_03":
            pub.publish(String("scene_03"))
        elif event == "scene_04":
            pub.publish(String("scene_04"))
        elif event == "scene_05":
            pub.publish(String("scene_05"))
        elif event == "scene_06":
            pub.publish(String("scene_06"))
        elif event == "scene_07":
            pub.publish(String("scene_07"))
        elif event == "scene_08":
            pub.publish(String("scene_08"))
        elif event == "scene_09":
            pub.publish(String("scene_09"))
        elif event == "scene_10":
            pub.publish(String("scene_10"))
        elif event == "scene_11":
            pub.publish(String("scene_11"))
        elif event == "scene_12":
            pub.publish(String("scene_12"))
        elif event == "scene_13":
            pub.publish(String("scene_13"))
        elif event == "scene_14":
            pub.publish(String("scene_14"))
        elif event == "Kinect":
            pub.publish(String("kinect"))

        rate.sleep() # Sleeps to if time < rate




if __name__ == '__main__':
    main()
