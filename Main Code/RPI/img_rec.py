import sys
sys.path.append('/usr/lib/python3/dist-packages')

from picamera import PiCamera
from sendtopc import *

def imgRec():
    camera = PiCamera()
    print("Taking photo 1...")
    camera.capture('/home/mdp2022/shared/image/image1.jpg')
    camera.close()

    label = sendImgToPC()
    return label
