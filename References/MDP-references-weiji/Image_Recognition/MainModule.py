import cv2

import argparse
import os
import sys
from pathlib import Path

import cv2
import torch
import torch.backends.cudnn as cudnn

from detect_test import run
from detect_test import parse_opt

from models.common import DetectMultiBackend
from utils.datasets import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams
from utils.general import (LOGGER, check_file, check_img_size, check_imshow, check_requirements, colorstr, is_ascii,
                           increment_path, non_max_suppression, print_args, scale_coords, strip_optimizer, xyxy2xywh,
                            apply_classifier,set_logging)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, time_sync

import time

# Open the device at the ID 0
# Use the camera ID based on
# /dev/videoID needed
cap = cv2.VideoCapture(0)

#Check if camera was opened correctly
if not (cap.isOpened()):
    print("Could not open video device")


#Set the resolution, not very impt
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

result = {'A': 'A', 'B': 'B', 'C': 'C', 'D': 'D', 'E': 'E', 'F': 'F', 'G': 'G', 'H': 'H',
             'S': 'S', 'T': 'T', 'U': 'U', 'V': 'V', 'W': 'W', 'X': 'X', 'Y': 'Y', 'Z': 'Z',
             'Circle': 'Circle', 'Target': 'Target', 'Down': 'Down', '8': '8', '5': '5', '4': '4',
             'Left': 'Left', '9': '9', '1': '1', 'Right': 'Right', '7': '7', '6': '6', '3': '3', '2': '2',
             'Up': 'Up'}

# Call main function which returns a label, this label would represent the class
def main(opt):
    check_requirements(exclude=('tensorboard', 'thop'))
    label = run(**vars(opt))
    return label
    
# Capture frame-by-frame
count = 10   # This would make sure we can capture every 20 frames i think

actual_img_count_without_duplicate = 0

seen_labels = []  # If we have already detected the image before, just skip it
seen_images = []  # Not required

while(True):
    ret, frame = cap.read()

    # Display the resulting frame
    if count%10==0:
        cv2.imshow("preview",frame)

        file_path1 = "C:/Users/micha/OneDrive/Desktop/PRED/content/PRED/images/outputImage.jpg"  # We'll constantly override this so as to ensure we only have 1 image file in the folder at all times
        
        # actual_count += 1 # Not needed anymore
        
        cv2.imwrite(file_path1, frame)

     #   time.sleep(2)
        if len(os.listdir( "C:/Users/micha/OneDrive/Desktop/PRED/content/PRED/images" ) ) == 0:   # Just to make sure the folder isnt empty, else there'l be an error
            continue
        else:
            if __name__ == "__main__":
                opt = parse_opt()
                label = main(opt)
        #        print("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS")
                print("I SEE", label)
                # seen_images.append(file_path1)
                if (label == "null"):
                    continue
                if (label not in seen_labels):
                    seen_labels.append(label)
                 #   file_path2 = "C:/Users/micha/OneDrive/Desktop/PRED/content/PRED/collage_actual/ToutputImage{0}.jpg".format(actual_img_count_without_duplicate)
        #            actual_img_count_without_duplicate += 1
         #           cv2.imwrite(file_path2, frame)
    count += 1
    

    cr = cv2.waitKey(1)
    if cr == 27:
        break
    """
    #Waits for a user input to quit the application
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break
    """
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

import cv2
import os
from PIL import Image

def collage(width, height, list_of_images):
    rows = 2
    n = len(list_of_images)
#    if n % 2 == 1:
#        list_of_images.append('black.jpg')
    cols = int(n / rows) + int(n % rows > 0)
    size = width, height
    new_im = Image.new('RGB', (width*cols, height*rows))
    ims = []
    for p in list_of_images:
        im = Image.open(p)
        im.thumbnail(size)
        ims.append(im)
    i = 0
    x = 0
    y = 0
    for col in range(cols):
        for row in range(rows):
            new_im.paste(ims[i], (x, y))
            i += 1
            y += height
        x += width
        y = 0

    
    new_im.save("collage.jpg")
    cv2.imshow('collage',new_im)

folder = "C:/Users/micha/OneDrive/Desktop/PRED/content/PRED/collage_actual"
file_names = []

for filename in os.listdir(folder):
    file_names.append( os.path.join(folder,filename) )
         
print(file_names)

collage(640, 480, file_names)
    

