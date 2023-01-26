#!/usr/bin/env python3

import pyglet
import os
import rospy
from gtts import gTTS
from time import sleep
from dora_the_mug_finder_msg.msg import Object


def msgReceivedCallback(data):

    # Get properties lists from the message
    lengths = data.length
    heights = data.height
    widths = data.width
    colors = data.color
    scene = data.scene.data

    scene_parts = scene.split('_')
    scene_name =scene_parts[0]
    scene_number = scene_parts[-1]

    # Initial text
    text = 'The ' + str(scene_name) + str(scene_number) +  ' has ' + str(len(lengths)) + ' objects.'
            
    # Description of each object
    for object_idx , length in enumerate(lengths):
        print(object_idx)

        object_idx_length = lengths[object_idx].data
        object_idx_width = widths[object_idx].data
        object_idx_height = heights[object_idx].data
        #object_idx_color = colors[object_idx].data
    
        text = text + 'Object number ' + str(object_idx+1) + ' has dimensions of ' + str(object_idx_length) + ' per ' + str(object_idx_width) + ' per ' + str(object_idx_height)

        if len(colors)== 0:
            text = text + '. There is not information about the color.'

        else: 
            text = text + ' and it is ' + str(object_idx_color) + ' .'
        
        # Start Google TextToSpeech with the generated text with the voice with indian accent
        tts = gTTS(text=text, lang='en',  tld='co.in')
        
        # Create a temporary audio file with our text
        filename = '/tmp/temp.mp3'
        tts.save(filename)

        # Play the audio file 
        music = pyglet.media.load(filename, streaming=False)
        music.play()

        sleep(music.duration) # Prevents from killing #! Not sure if necessary, we may want to kill during audio playback
        os.remove(filename) # Remove temporary file

def texttospeech():
    
    # Initialization of a ROS node
    rospy.init_node('objects_properties', anonymous=False)
    
    # Initialize subscriber
    rospy.Subscriber('objects_publisher', Object,  msgReceivedCallback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin() 
 
if __name__ == "__main__":
    texttospeech()