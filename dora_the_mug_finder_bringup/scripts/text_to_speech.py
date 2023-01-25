#!/usr/bin/env python3

import pyglet
import os
import rospy
from gtts import gTTS
from time import sleep
from dora_the_mug_finder_msg.msg import Object


def msgReceivedCallback():

    # Get properties lists from the message
    length = Object['length']
    height = Object['height']
    width = Object['width']
    color = Object['color']
    scene = Object['scene']
    
    #! Test, if none we try the gtts either way
    if Object is None:
            length = 12
            height = 0.5
            width = 3.2
            scene = 'scene_01'
            
    # Description of each object
    for object_idx in length:

        object_idx_length = length[object_idx]
        object_idx_width = width[object_idx]
        object_idx_height = height[object_idx]
        object_idx_color = color[object_idx]

        # Initial text
        text = 'The ' + str(scene) +  ' has ' + str(len(length)) + ' objects.'
    
        text = text + 'Object number ' + str(object_idx+1) + ' has dimensions of ' + str(object_idx_length) + ' per ' + str(object_idx_width) + ' per ' + str(object_idx_height)

        if color is None:
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