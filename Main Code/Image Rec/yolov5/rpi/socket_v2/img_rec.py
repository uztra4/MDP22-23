import sys
sys.path.append('/usr/lib/python3/dist-packages')

from picamera import PiCamera
from sendtopc import *

camera = PiCamera()
print("Taking photo...")
camera.capture('/home/mdp2022/images/image.jpg')

print("photo taken")
sendImgToPC()