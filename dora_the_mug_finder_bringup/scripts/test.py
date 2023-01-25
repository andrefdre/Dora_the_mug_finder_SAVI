#!/usr/bin/env python3

import pyglet
import os
import rospy
from gtts import gTTS
from time import sleep
from dora_the_mug_finder_msg.msg import Object



def texttospeech():

    length = 12
    height = 0.5
    width = 3.2
    scene = 'scene_01'
    
    # Initial text
    text = 'The ' + str(scene) +  ' has ' + str(length) + ' objects.'
    
    text = text + 'Object number ' + str(1) + ' has dimensions of ' + str(length) + ' per ' + str(width) + ' per ' + str(height)
    text = text + '. The object color is fucking blue.'
    text = text + 'I really ate 12 giant mushrooms. They were astonishingly delicious. I really want 45 dozens more.  '
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

if __name__ == "__main__":
    texttospeech()