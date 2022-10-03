import sys
sys.path.append('/usr/lib/python3/dist-packages')

from picamera import PiCamera
from sendtopc import *

def imgRec():
    camera = PiCamera()
    camera.resolution=(615,462)
    print("Taking photo 1...")
    camera.capture('/home/mdp2022/shared/image/image1.jpg')
    camera.close()

    label = sendImgToPC()
    print(label)
    return label
