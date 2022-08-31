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

@torch.no_grad()
def run(
        weights=ROOT / 'yolov5s.pt',  # model.pt path(s)
        source=ROOT / 'data/images',  # file/dir/URL/glob, 0 for webcam
        data=ROOT / 'data/coco128.yaml',  # dataset.yaml path
        imgsz=(640, 640),  # inference size (height, width)
        conf_thres=0.25,  # confidence threshold
        iou_thres=0.45,  # NMS IOU threshold
        max_det=1000,  # maximum detections per image
        device='',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        view_img=False,  # show results
        save_txt=False,  # save results to *.txt
        save_conf=False,  # save confidences in --save-txt labels
        save_crop=False,  # save cropped prediction boxes
        nosave=False,  # do not save images/videos
        classes=None,  # filter by class: --class 0, or --class 0 2 3
        agnostic_nms=False,  # class-agnostic NMS
        augment=False,  # augmented inference
        visualize=False,  # visualize features
        update=False,  # update all models
        project=ROOT / 'runs/detect',  # save results to project/name
        name='exp',  # save results to project/name
        exist_ok=False,  # existing project/name ok, do not increment
        line_thickness=3,  # bounding box thickness (pixels)
        hide_labels=False,  # hide labels
        hide_conf=False,  # hide confidences
        half=False,  # use FP16 half-precision inference
        dnn=False,  # use OpenCV DNN for ONNX inference,
        img_array = []
        ):

    # Give it a fake source
    # This is required simply to ensure code that it doesnt screw up code that might use the variable "source" below
    source = ""
    
    # Print the array to check if it exists
    #print(img_array)
    
##################
    # ADDED ON 6/2/2022
    # CHANGE THIS TO THE RESPECTIVE FILE PATHS, AND VARIABLES
    
    #source = str("C:/Users/micha/OneDrive/Desktop/PRED/content/PRED")
    weights_src = ROOT / "best.pt"
    data_src = ROOT /"data/coco128.yaml"
    conf_thresh = 0.4
    save_folder = ROOT / "collage_actual"
    #source_folder = WHERE RPI STORE THE SS TAKEN. We delete pic from this folder when inference has been completed
##################

    # Print source to check where its coming from
    #Removed cos we dont need it abymore, as we techically dont have a source anymore
    #print(source)

    
    save_img = not nosave and not source.endswith('.txt')  # save inference images
    is_file = Path(source).suffix[1:] in (IMG_FORMATS + VID_FORMATS)
    is_url = source.lower().startswith(('rtsp://', 'rtmp://', 'http://', 'https://'))
    webcam = source.isnumeric() or source.endswith('.txt') or (is_url and not is_file)
    if is_url and is_file:
        source = check_file(source)  # download

    # Directories
    save_dir = increment_path(Path(project) / name, exist_ok=exist_ok)  # increment run
    (save_dir / 'labels' if save_txt else save_dir).mkdir(parents=True, exist_ok=True)  # make dir

    # Load model
    device = select_device(device)
    model = DetectMultiBackend(weights_src, device=device, dnn=dnn, data=data_src)
    stride, names, pt, jit, onnx, engine = model.stride, model.names, model.pt, model.jit, model.onnx, model.engine
    imgsz = check_img_size(imgsz, s=stride)  # check image size

    # Half
    half &= (pt or jit or onnx or engine) and device.type != 'cpu'  # FP16 supported on limited backends with CUDA
    if pt or jit:
        model.model.half() if half else model.model.float()


    # Run inference
    model.warmup(imgsz=(1, 3, *imgsz), half=half)  # warmup
    dt, seen = [0.0, 0.0, 0.0], 0

    # ADDED ON 9/2/2022
    im, im0s = convert_array(img_array, img_size=640, stride=32, auto=True)
    
   # for path, im, im0s, vid_cap, s in dataset:
        # path is nothing
        # im is 480
        # im0s is 640
    print(len(im0s[0]))
    t1 = time_sync()
    im = torch.from_numpy(im).to(device)
    im = im.half() if half else im.float()  # uint8 to fp16/32
    im /= 255  # 0 - 255 to 0.0 - 1.0
    if len(im.shape) == 3:
        im = im[None]  # expand for batch dim
    t2 = time_sync()
    dt[0] += t2 - t1

    # JUST ANYHOW PUT path for now
    path = " "
    # Inference
    visualize = increment_path(save_dir / Path(path).stem, mkdir=True) if visualize else False
    pred = model(im, augment=augment, visualize=visualize)
    t3 = time_sync()
    dt[1] += t3 - t2
   
    # NMS
    pred = non_max_suppression(pred, conf_thresh, iou_thres, classes, agnostic_nms, max_det=max_det)
    dt[2] += time_sync() - t3

    # Second-stage classifier (optional)
    # pred = utils.general.apply_classifier(pred, classifier_model, im, im0s)

    # Process predictions
    for i, det in enumerate(pred):  # per image
        seen += 1
        if webcam:  # batch_size >= 1
            p, im0, frame = path[i], im0s[i].copy(), dataset.count
            s += f'{i}: '
        else:
           # p, im0, frame = path, im0s.copy(), getattr(dataset, 'frame', 0)
           im0 = im0s.copy()

           p = " "
      #      p = Path(p)  # to Path
      #      save_path = str(save_dir / p.name)  # im.jpg
       #     txt_path = str(save_dir / 'labels' / p.stem) + ('' if dataset.mode == 'image' else f'_{frame}')  # im.txt
        #    s += '%gx%g ' % im.shape[2:]  # print string
       #    gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
       #     imc = im0.copy() if save_crop else im0  # for save_crop
           annotator = Annotator(im0, line_width=line_thickness, example=str(names))

        o = 'null'
            
        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

            # Print results
            s = "1"
            for c in det[:, -1].unique():
                n = (det[:, -1] == c).sum()  # detections per class
                s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string
                o = f"{names[int(c)]}"
                    
            img_id = {'A': '20', 'B': '21', 'C': '22', 'D': '23',
                          'E': '24', 'F': '25', 'G': '26', 'H': '27',
                          'S': '28', 'T': '29', 'U': '30', 'V': '31',
                          'W': '32', 'X': '33', 'Y': '34', 'Z': '35',
                          'Circle' : '40', 'Target': '420', 'Down': '37', '8': '18',
                          '5': '15', '4': '14', 'Left': '39', '9': '19', '1': '11',
                          'Right': '38', '7': '17', '6': '16', '3': '13', '2': '12',
                          'Up': '36'}
                
            # Write results
            for *xyxy, conf, cls in reversed(det):
            #        if save_txt:  # Write to file
             #           xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
             #           line = (cls, *xywh, conf) if save_conf else (cls, *xywh)  # label format
              #          with open(txt_path + '.txt', 'a') as f:
              #              f.write(('%g ' * len(line)).rstrip() % line + '\n')

                if save_img or save_crop or view_img:  # Add bbox to image
                    c = int(cls)  # integer class
                    label = None if hide_labels else (names[c] if hide_conf else f'{names[c]} id:{img_id[names[c]]} {conf:.2f}')

                    # ADDED ON 6/2/2022 # Sends RPI stuff
                    if (1):
                        prob = conf.numpy()
                        prob = np.round(prob,2)
                        send_rpi_id(img_id[names[c]] + " " + str(prob))
                        
                    annotator.box_label(xyxy, label, color=colors(c, True))
                    if save_crop:
                        save_one_box(xyxy, imc, file=save_dir / 'crops' / names[c] / f'{p.stem}.jpg', BGR=True)


            # Print time (inference-only)
            # LOGGER.info(f'{s}Done. ({t3 - t2:.3f}s)')
        print(f'{s}Done. ({t3 - t2:.3f}s)')
            
            # Stream results
        im0 = annotator.result()
        if view_img:
            cv2.imshow(str(p), im0)
            cv2.waitKey(3)  # 3 millisecond

        # Save image if theres a detection
        if o != 'null':
            img_name = str(names[c]) + "_" +str(img_id[names[c]]) + "_" + str(prob) + ".jpg"
            save_it = os.path.join(save_folder,img_name)
            cv2.imwrite(save_it, im0)
            
            # if o != 'null' and o != 'Target':
            #    cv2.imwrite(save_it, im0)
            #    img_count = 0
            #    while os.path.exists("img%s.jpg" % img_count):
            #        img_count += 1
            #    cv2.imwrite(save_path, im0)           

    # Print results
    t = tuple(x / seen * 1E3 for x in dt)  # speeds per image
    # LOGGER.info(f'Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS per image at shape {(1, 3, *imgsz)}' % t)
    print(f'Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS per image at shape {(1, 3, *imgsz)}' % t)
    if save_txt or save_img:
        s = f"\n{len(list(save_dir.glob('labels/*.txt')))} labels saved to {save_dir / 'labels'}" if save_txt else ''
        LOGGER.info(f"Results saved to {colorstr('bold', save_dir)}{s}")
    if update:
        strip_optimizer(weights_src)  # update model (to fix SourceChangeWarning)

    result = {'A': 'A', 'B': 'B', 'C': 'C', 'D': 'D', 'E': 'E', 'F': 'F', 'G': 'G', 'H': 'H',
             'S': 'S', 'T': 'T', 'U': 'U', 'V': 'V', 'W': 'W', 'X': 'X', 'Y': 'Y', 'Z': 'Z',
             'Circle': 'Circle', 'Target': 'Target', 'Down': 'Down', '8': '8', '5': '5', '4': '4',
             'Left': 'Left', '9': '9', '1': '1', 'Right': 'Right', '7': '7', '6': '6', '3': '3', '2': '2',
             'Up': 'Up'}
    try:
        return result[o]
    except (ValueError, Exception):
        return 'null'

# To allow us to pass in command line commands that'll map to the variables
# We dont need to do much for this, but we need to keep this function for it to work
def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default=ROOT / 'yolov5s.pt', help='model path(s)')
    parser.add_argument('--source', type=str, default=ROOT / 'data/images', help='file/dir/URL/glob, 0 for webcam')
    parser.add_argument('--data', type=str, default=ROOT / 'data/coco128.yaml', help='(optional) dataset.yaml path')
    parser.add_argument('--imgsz', '--img', '--img-size', nargs='+', type=int, default=[640], help='inference size h,w')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='NMS IoU threshold')
    parser.add_argument('--max-det', type=int, default=1000, help='maximum detections per image')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', help='show results')
    parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
    parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
    parser.add_argument('--save-crop', action='store_true', help='save cropped prediction boxes')
    parser.add_argument('--nosave', action='store_true', help='do not save images/videos')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --classes 0, or --classes 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--visualize', action='store_true', help='visualize features')
    parser.add_argument('--update', action='store_true', help='update all models')
    parser.add_argument('--project', default=ROOT / 'runs/detect', help='save results to project/name')
    parser.add_argument('--name', default='exp', help='save results to project/name')
    parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
    parser.add_argument('--line-thickness', default=3, type=int, help='bounding box thickness (pixels)')
    parser.add_argument('--hide-labels', default=False, action='store_true', help='hide labels')
    parser.add_argument('--hide-conf', default=False, action='store_true', help='hide confidences')
    parser.add_argument('--half', action='store_true', help='use FP16 half-precision inference')
    parser.add_argument('--dnn', action='store_true', help='use OpenCV DNN for ONNX inference')
   # parser.add_argument('--img_array', action='store_true', help='use OpenCV DNN for ONNX inference')
    opt = parser.parse_args()
    opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # expand
    print_args(FILE.stem, opt)
    return opt

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



# This is the main function, it takes two inputs
# opt is the parser
# arr is the numpy array we wanna pass through
# label returns the Class name NOTE: This is not the image ID, it it the class name.
# If you want the class ID can just use the dictionary in the code above somwhere
def main(arr):
    check_requirements(exclude=('tensorboard', 'thop'))
    opt = parse_opt()
    label = run(**vars(opt), img_array = arr)
    print("THIS IMAGE IS DETECTED. " + label  + " .THIS IS JUST TO SHOW THAT IS SUCCESFFUL")



# This just calls the main function
if __name__ == "__main__":
   
    # Test 1
    image_numpy = cv2.imread("C:/Users/micha/OneDrive/Desktop/PRED/content/PRED/detect_this.jpg")

    # Test 2
    #image_numpy = cv2.imread("D:/micha/Michael/NTU/NTU Y2/Sem 2/CZ3004_MDP/YOLOv5_1.1/testImgs/H/H_test_1.jpg")

    # Just need to pass in the numpy array (image_numpy) here and we're good to go
    main(arr = image_numpy)  # We can just pass in a numpy array as the second argument

