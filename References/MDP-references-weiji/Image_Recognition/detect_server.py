# YOLOv5 🚀 by Ultralytics, GPL-3.0 license
"""
Run inference on images, videos, directories, streams, etc.

Usage - sources:
    $ python path/to/detect.py --weights yolov5s.pt --source 0              # webcam
                                                             img.jpg        # image
                                                             vid.mp4        # video
                                                             path/          # directory
                                                             path/*.jpg     # glob
                                                             'https://youtu.be/Zgi9g1ksQHc'  # YouTube
                                                             'rtsp://example.com/media.mp4'  # RTSP, RTMP, HTTP stream

Usage - formats:
    $ python path/to/detect.py --weights yolov5s.pt                 # PyTorch
                                         yolov5s.torchscript        # TorchScript
                                         yolov5s.onnx               # ONNX Runtime or OpenCV DNN with --dnn
                                         yolov5s.xml                # OpenVINO
                                         yolov5s.engine             # TensorRT
                                         yolov5s.mlmodel            # CoreML (MacOS-only)
                                         yolov5s_saved_model        # TensorFlow SavedModel
                                         yolov5s.pb                 # TensorFlow GraphDef
                                         yolov5s.tflite             # TensorFlow Lite
                                         yolov5s_edgetpu.tflite     # TensorFlow Edge TPU
"""

import argparse
import os
import sys
from pathlib import Path

import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np
import glob


FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative



from models.common import DetectMultiBackend
from utils.datasets import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams
from utils.general import (LOGGER, check_file, check_img_size, check_imshow, check_requirements, colorstr, is_ascii,
                           increment_path, non_max_suppression, print_args, scale_coords, strip_optimizer, xyxy2xywh,
                            apply_classifier,set_logging)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, time_sync
from utils.augmentations import Albumentations, augment_hsv, copy_paste, letterbox, mixup, random_perspective

img_id = {'A': '20', 'B': '21', 'C': '22', 'D': '23',
                            'E': '24', 'F': '25', 'G': '26', 'H': '27',
                            'S': '28', 'T': '29', 'U': '30', 'V': '31',
                            'W': '32', 'X': '33', 'Y': '34', 'Z': '35',
                            'Circle' : '40', 'Target': '420', 'Down': '37', '8': '18',
                            '5': '15', '4': '14', 'Left': '39', '9': '19', '1': '11',
                            'Right': '38', '7': '17', '6': '16', '3': '13', '2': '12',
                            'Up': '36', 'null': '-1'}

# A function that is essential to convert the numpy array so the model can perform predictions
def convert_array(img_array, img_size=640, stride=32, auto=True):

    img0 = img_array  # BGR
    # Padded resize
    img = letterbox(img0, (640, 480), stride=32, auto=True)[0]
    # Convert
    img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
    img = np.ascontiguousarray(img)

    return img, img0

# CHANGE THIS TO THE COMMAND TO TALK/SEND DATA TO RPI
# Not sure if this is still needed
def send_rpi_id(id_number):
    print("sending to rpi that we have seen image with ID of ", id_number)

class DetectServer:
    def __init__(self, device='',dnn=False, imgsz=(640, 640), half=False, augment=False,
        project=ROOT / 'runs/detect',  # save results to project/name
        visualize=False,  # visualize features
        save_txt=False,  # save results to *.txt
        name='exp',  # save results to project/name
        classes=None
                 ) -> None:

        self.augment = augment
        self.classes = None
        self.visualize = visualize
        print("AAA", ROOT)
        #source = str("C:/Users/micha/OneDrive/Desktop/PRED/content/PRED")
        weights_src = ROOT / "best.pt"
        data_src = ROOT /"data/coco128.yaml"
        self.conf_thresh = 0.4
        self.save_folder = ROOT / "collage_actual"
        self.iou_thres=0.45
        self.half = half
        
        # Load model
        self.device = select_device(device)
        self.model = DetectMultiBackend(weights_src, device=self.device, dnn=dnn, data=data_src)

        stride, self.names, pt, jit, onnx, engine = self.model.stride, self.model.names, self.model.pt, self.model.jit, self.model.onnx, self.model.engine
        imgsz = check_img_size(imgsz, s=stride)  # check image size

        # Half
        self.half &= (pt or jit or onnx or engine) and self.device.type != 'cpu'  # FP16 supported on limited backends with CUDA
        if pt or jit:
            self.model.model.half() if half else self.model.model.float()

        self.model.warmup(imgsz=(1, 3, *imgsz), half=half)  # warmup

        self.save_dir = increment_path(Path(project) / name, exist_ok=False)  # increment run
        (self.save_dir / 'labels' if save_txt else self.save_dir).mkdir(parents=True, exist_ok=True)  # make dir


    @torch.no_grad()
    def detect(self, img_array,
               line_thickness=3,  # bounding box thickness (pixels)
               max_det=1000,  # maximum detections per image
               hide_labels=False,  # hide labels
               hide_conf=False,  # hide confidences
               view_img=False,  # show results
               ):
        dt, seen = [0.0, 0.0, 0.0], 0

        # ADDED ON 9/2/2022
        im, im0s = convert_array(img_array, img_size=640, stride=32, auto=True)

        t1 = time_sync()
        im = torch.from_numpy(im).to(self.device)
        im = im.half() if self.half else im.float()  # uint8 to fp16/32
        im /= 255  # 0 - 255 to 0.0 - 1.0
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim
        t2 = time_sync()
        dt[0] += t2 - t1

        # JUST ANYHOW PUT path for now
        path = " "
        # Inference
        visualize = increment_path(self.save_dir / Path(path).stem, mkdir=True) if self.visualize else False
        pred = self.model(im, augment=self.augment, visualize=visualize)
        t3 = time_sync()
        dt[1] += t3 - t2

        # NMS
        pred = non_max_suppression(pred, self.conf_thresh, self.iou_thres, self.classes, False, max_det=max_det)
        dt[2] += time_sync() - t3

        # Second-stage classifier (optional)
        # pred = utils.general.apply_classifier(pred, classifier_model, im, im0s)

        detect_array = []
        predict_array = []
        image_list = []
        # Process predictions
        for i, det in enumerate(pred):  # per image
            seen += 1
            # p, im0, frame = path, im0s.copy(), getattr(dataset, 'frame', 0)
            im0 = im0s.copy()

            p = " "
            annotator = Annotator(im0, line_width=line_thickness, example=str(self.names))

            o = 'null'

            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                s = "1"
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "  # add to string
                    o = f"{self.names[int(c)]}"
                    detect_array.append(o)

                # Write results
                for *xyxy, conf, cls in reversed(det):
                    c = int(cls)  # integer class
                    label = None if hide_labels else (self.names[c] if hide_conf else f'{self.names[c]} id:{img_id[self.names[c]]} {conf:.2f}')

                    # ADDED ON 6/2/2022 # Sends RPI stuff
                    prob = conf.numpy()
                    prob = np.round(prob,2)
                    send_rpi_id(img_id[self.names[c]] + " " + str(prob))
                    
                    annotator.box_label(xyxy, label, color=colors(c, True))
                    predict_array.append(prob)

            im0 = annotator.result()
            image_list.append(im0)
            if view_img:
                cv2.imshow(str(p), im0)
                cv2.waitKey(0)

            # Save image if theres a detection
            if o != 'null' and o != 'Target':
                img_name = str(img_id[self.names[c]]) +  "_" + str(prob) + ".jpg"
                save_it = os.path.join(self.save_folder,img_name)
                exists_file = os.path.join(self.save_folder,"*.jpg")

                head_tail_1 = os.path.split(save_it)
                files = glob.glob(exists_file)
                id_detected = img_id[self.names[c]]

                flag = 0
                for i in files:
                    head_tail_2 = os.path.split(i)
                    tail2 = head_tail_2[1]
                    id_file = float(tail2[0:2])
                    if id_detected == id_file:
                        prob_old = float(tail2[3:7])
                        if (prob > prob_old):
                            flag = 1
                            cv2.imwrite(save_it, im0)
                            break
                if (flag == 0):
                    cv2.imwrite(save_it, im0)

                    

        # send this array to RPI detect_array
        result_dict = {}
        image_dict = {}
        count = 0
        for i in predict_array:
            result_dict[i] = detect_array[count]
            image_dict[i] = image_list[count]
            count = count + 1
        order_of_list = sorted(result_dict)[::-1]
        
        detect_array = []
        sorted_images = []
        for i in range(len(order_of_list)):
            id = img_id[result_dict[order_of_list[i]]]
            detect_array.append(id)
            sorted_images.append(image_dict[order_of_list[i]])
                                
        return detect_array, sorted_images

# This just calls the main function
if __name__ == "__main__":
   

    # Test 1
    
    image_numpy = cv2.imread("/home/aru/temp.jpg")

    server = DetectServer()
    server.detect(image_numpy)
