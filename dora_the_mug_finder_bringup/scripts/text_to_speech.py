#!/usr/bin/env python3

import pyglet
import os
import rospy
from gtts import gTTS
from time import sleep
from dora_the_mug_finder_msg.msg import Object, Classes
from functools import partial

class ROSHandler:

    def __init__(self):
        self.scenes_list = []
        self.first_time = True

    def msgReceivedCallback(self,data):
        # Get properties from the message
        lengths = data.length
        heights = data.height
        widths = data.width
        colors = data.color
        scene = data.scene.data
        objects_number = int(data.objects_number.data)

        # Splits scene name for better reading. Separates the text from the number
        scene_parts = scene.split('_')
        scene_name =scene_parts[0]
        scene_number = scene_parts[-1]

        if self.first_time:
            first_text = 'Holla, soy Dora la exploradora. Vámos a empezar nuestra aventura?'
            # Start Google TextToSpeech with the generated text with the voice with indian accent
            tts = gTTS(text=first_text, lang='es',  tld='com.mx')
            
            # Create a temporary audio file with our text
            filename = '/tmp/temp.mp3'
            tts.save(filename)

            # Play the audio file 
            music = pyglet.media.load(filename, streaming=False)
            music.play()
            sleep(music.duration) # Prevents from killing #! Not sure if necessary, we may want to kill during audio playback
            os.remove(filename) # Remove temporary file
            self.first_time = False


        if scene_number in self.scenes_list:
            return
        
        elif len(lengths) <=2:
            return
        
        else:
            self.scenes_list.append(scene_number)
            object_idx_list = []
            # Initial text
            int_text = 'The ' + str(scene_name) + str(scene_number) +  ' has ' + str(objects_number) + ' objects.'
            
            # Start Google TextToSpeech with the generated text with the voice with indian accent
            tts = gTTS(text=int_text, lang='es',  tld='com.mx')
            
            # Create a temporary audio file with our text
            filename = '/tmp/temp.mp3'
            tts.save(filename)

            # Play the audio file 
            music = pyglet.media.load(filename, streaming=False)
            music.play()

            sleep(music.duration) # Prevents from killing #! Not sure if necessary, we may want to kill during audio playback
            os.remove(filename) # Remove temporary file

            # Description of each object
            for object_idx in range(0, objects_number):
                
                if object_idx in object_idx_list:
                    return
                
                else:
                    object_idx_length = int(lengths[object_idx].data)
                    object_idx_width = int(widths[object_idx].data)
                    object_idx_height =int(heights[object_idx].data)
                    object_idx_color = colors[object_idx].data
                    
                    text = 'Object number ' + str(object_idx+1) + ' has dimensions of ' + str(object_idx_length) + ' per ' + str(object_idx_width) + ' per ' + str(object_idx_height) + ' centimeters and it is ' + str(object_idx_color) + ' .'

                    # Start Google TextToSpeech with the generated text with the voice with indian accent
                    tts = gTTS(text=text, lang='es',  tld='com.mx')
                    
                    # Create a temporary audio file with our text
                    filename = '/tmp/temp.mp3'
                    tts.save(filename)

                    # Play the audio file 
                    music = pyglet.media.load(filename, streaming=False)
                    music.play()

                    sleep(music.duration) # Prevents from killing #! Not sure if necessary, we may want to kill during audio playback
                    os.remove(filename) # Remove temporary file
                    object_idx_list.append(object_idx)

def texttospeech():
    # Initialization of a ROS node
    rospy.init_node('objects_properties', anonymous=False)
    
    ros_handler=ROSHandler()
    # Initialize subscriber
    rospy.Subscriber('objects_publisher', Object,  ros_handler.msgReceivedCallback)
    
    # spin() simply keeps python from exiting until this node is stopped  
    rospy.spin() 
 

if __name__ == "__main__":
    texttospeech()
