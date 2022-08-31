from image_client import ImageClient
from detect_server import DetectServer
from PIL import Image
import cv2
import numpy as np

server = DetectServer()
image_client = ImageClient("192.168.15.15", 50000)
tiled_img_arr = []

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
        im = Image.fromarray(p)
        im.thumbnail(size)
        ims.append(im)
    i = 0
    x = 0
    y = 0
    for col in range(cols):
        for row in range(rows):
            try:
                new_im.paste(ims[i].resize(200, 200), (x, y))
                i += 1
                y += height
            except:
                pass
        x += width
        y = 0

    cv2.imshow('collage',np.array(new_im))
    cv2.waitKey(1)


def detect_callback(msg):
    global tiled_img_arr

    output, imgs = server.detect(msg)
    print(output)
    if not output:
        image_client.send("None".encode())
        pass
    else:
        image_client.send(output[0].encode())
        tiled_img_arr.append(imgs[0])
        collage(1000, 1000, tiled_img_arr)
        
image_client.recv_callback = detect_callback
image_client.loop()

