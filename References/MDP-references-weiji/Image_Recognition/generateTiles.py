import cv2
import os
from PIL import Image
import numpy as np

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
            try:
                new_im.paste(ims[i], (x, y))
                i += 1
                y += height
            except:
                pass
        x += width
        y = 0

    new_im.save("collage.jpg")
    cv2.imshow('collage',np.array(new_im))
    cv2.waitKey(0)

folder = "./collage_actual"
file_names = []

for filename in os.listdir(folder):
    file_names.append( os.path.join(folder,filename) )
         
print(file_names)

collage(640, 480, file_names)
