# YOLOv5 🚀 by Ultralytics, GPL-3.0 license
"""

실행 명령어: python3 detect.py --source realsense --weights chkpt/yolov5s.pt --nosave --view-img --augment


python3 detect_with_ros3.py --source realsense --weights chkpt/yolov5s.pt --nosave --view-img --augment


리얼센스와 yolov5 사용


Run YOLOv5 detection inference on images, videos, directories, globs, YouTube, webcam, streams, etc.

Usage - sources:
    $ python detect.py --weights yolov5s.pt --source 0                               # webcam
                                                     img.jpg                         # image
                                                     vid.mp4                         # video
                                                     path/                           # directory
                                                     'path/*.jpg'                    # glob
                                                     'https://youtu.be/Zgi9g1ksQHc'  # YouTube
                                                     'rtsp://example.com/media.mp4'  # RTSP, RTMP, HTTP stream

Usage - formats:
    $ python detect.py --weights yolov5s.pt                 # PyTorch
                                 yolov5s.torchscript        # TorchScript
                                 yolov5s.onnx               # ONNX Runtime or OpenCV DNN with --dnn
                                 yolov5s.xml                # OpenVINO
                                 yolov5s.engine             # TensorRT
                                 yolov5s.mlmodel            # CoreML (macOS-only)
                                 yolov5s_saved_model        # TensorFlow SavedModel
                                 yolov5s.pb                 # TensorFlow GraphDef
                                 yolov5s.tflite             # TensorFlow Lite
                                 yolov5s_edgetpu.tflite     # TensorFlow Edge TPU
                                 yolov5s_paddle_model       # PaddlePaddle
"""

import argparse
import os
import platform
import sys
from pathlib import Path

import torch
import torch.backends.cudnn as cudnn

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
sys.path.append(str(FILE.parents[1])+'/yolov5_greencamp')  # yolov5 서브모듈 패스 넣어주기.
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from models.common import DetectMultiBackend
from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadScreenshots, LoadStreams
from utils.general import (LOGGER, Profile, check_file, check_img_size, check_imshow, check_requirements, colorstr, cv2,
                           increment_path, non_max_suppression, print_args, scale_boxes, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, smart_inference_mode

####
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2

import rospy
from rostopic import get_topic_type
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

from std_msgs.msg import String

class Yolov5Detector:
    def __init__(self,
            weights=ROOT / 'yolov5s.pt',  # model path or triton URL
            source=ROOT / 'data/images',  # file/dir/URL/glob/screen/0(webcam)
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
            dnn=False,  # use OpenCV DNN for ONNX inference
            vid_stride=1,  # video frame-rate stride
        ):
        source = str(source)
        if source != 'realsense':
            raise Exception('not realsense.')

        self.save_img = not nosave and not source.endswith('.txt')  # save inference images
        is_file = Path(source).suffix[1:] in (IMG_FORMATS + VID_FORMATS)
        is_url = source.lower().startswith(('rtsp://', 'rtmp://', 'http://', 'https://'))
        webcam = source.isnumeric() or source.endswith('.txt') or (is_url and not is_file)
        screenshot = source.lower().startswith('screen')

        if is_url and is_file:
            source = check_file(source)  # download

        # Directories
        save_dir = increment_path(Path(project) / name, exist_ok=exist_ok)  # increment run
        (save_dir / 'labels' if save_txt else save_dir).mkdir(parents=True, exist_ok=True)  # make dir

        # Load model
        self.device = select_device(device)
        self.model = DetectMultiBackend(weights, device=self.device, dnn=dnn, data=data, fp16=half)
        self.stride, self.names, self.pt = self.model.stride, self.model.names, self.model.pt
        
        self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, self.max_det = \
            conf_thres, iou_thres, classes, agnostic_nms, max_det
        self.visualize, self.hide_labels, self.hide_conf, self.augment = visualize, hide_labels, hide_conf, augment

        self.save_crop, self.view_img = save_crop, view_img
        
        self.imgsz = check_img_size(imgsz, s=self.stride)  # check image size

        # Streaming loop
        self.red_color = (0,0,255)

        cudnn.benchmark = True

        # Initialize subscriber to Image/CompressedImage topic
        input_image_type, input_image_topic, _ = get_topic_type("/DepthImageAlign", blocking = True)
        self.compressed_input = input_image_type == "sensor_msgs/CompressedImage"
        
        if self.compressed_input:
            self.image_sub = rospy.Subscriber(
                input_image_topic, CompressedImage, self.image_callback, queue_size=1
            )
        else:
            self.image_sub = rospy.Subscriber(
                input_image_topic, Image, self.image_callback, queue_size=1
            )

        self.mode_sub = rospy.Subscriber(
            "/mode", String, self.inference_callback, queue_size=1
        )

        self.flag = False

        self.bridge = CvBridge()

    def preprocess(self, img):
        """
        Adapted from yolov5/utils/datasets.py LoadStreams class
        """
        img0 = img.copy()
        img = np.array([letterbox(img, self.img_size, stride=self.stride, auto=self.pt)[0]])
        # Convert
        img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
        img = np.ascontiguousarray(img)

        return img, img0 

    def image_callback(self, ros_data):
        self.flag = True
        np_arr = np.fromstring(ros_data.data, np.uint8)
        compressed = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
        # print("image")

        # compressed = self.bridge.imgmsg_to_cv2(ros_data, ros_data.encoding)
        # import pdb; pdb.set_trace()
        self.image_np=compressed[:,:,:3]
        self.depth_np=compressed[:,:,3] 


    def inference_callback(self, data):
        ###########
        print("callback!")
        if (not self.flag):
            return
        print("start!")

        im0 = self.image_np.copy()
        img = self.image_np[np.newaxis, :, :, :]

        
        # # Stack
        img = np.stack(img, 0)

        # # Convert
        img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
        # channel ,height widht가 되야하고. rgb로 바꿔야함.
        img = np.ascontiguousarray(img)

        seen, windows, dt = 0, [], (Profile(), Profile(), Profile())
        #print(dt)
        with dt[0]:
            #print(img.shape) 
            im = torch.from_numpy(img).to(self.model.device)
            im = im.half() if self.model.fp16 else im.float()  # uint8 to fp16/32
            im /= 255  # 0 - 255 to 0.0 - 1.0
            if len(im.shape) == 3:
                im = im.unsqueeze(0)  # expand for batch dim
            #print("dt img : ",im.shape)
        # Inference
        with dt[1]:
            #print("dt 1")
            self.visualize = increment_path(save_dir / Path(path).stem, mkdir=True) if self.visualize else False

            # import pdb; pdb.set_trace()
            pred = self.model(im, augment=self.augment, visualize=self.visualize)
            #print("pred before : ",len(pred))

        # NMS
        with dt[2]:
            #print("dt 2")
            pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det)

        # Second-stage classifier (optional)
        # pred = utils.general.apply_classifier(pred, classifier_model, im, im0s)
        #print("here2")
        # Process predictions
        #print("pred shpae : ",len(pred))
        s = ''
        for i, det in enumerate(pred):  # per image
            s = ''
            seen += 1
            # if webcam:  # batch_size >= 1
            #     p, im0, frame = path[i], im0s[i].copy(), dataset.count
            #     s += f'{i}: '
            # else:
            #     p, im0, frame = path, im0s.copy(), getattr(dataset, 'frame', 0)

            # p = Path(p)  # to Path
            # save_path = str(save_dir / p.name)  # im.jpg
            # txt_path = str(save_dir / 'labels' / p.stem) + ('' if dataset.mode == 'image' else f'_{frame}')  # im.txt
            
            # im0는 원본이미지. numpy

            #im0 = img.copy()
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            
            annotator = Annotator(im0, line_width=3, example=str(self.names))
            xyxys = []
            if len(det):
                # Rescale boxes from img_size to im0 size
                #print(im.shape[2:], det[:, :4], im0.shape)
                det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, 5].unique():
                    n = (det[:, 5] == c).sum()  # detections per class
                    s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "  # add to string
                #print(reversed(det))
                # Write results
                for *xyxy, conf, cls in reversed(det):
                    xyxys.append(xyxy)
                    # xyxy에 네모상자 좌표가 담긴다.
                    #print(xyxy)
                    #print("for")
                    if self.save_img or self.save_crop or self.view_img:  # Add bbox to image
                        #print("save_img")
                        c = int(cls)  # integer class
                        label = None if self.hide_labels else (self.names[c] if self.hide_conf else f'{self.names[c]} {conf:.2f}')
                        annotator.box_label(xyxy, label, color=colors(c, True))

            # Stream results
            im0 = annotator.result()
            #print("im0 shpae : ",im0.shape)
            for xyxy in xyxys:
                    xy_coord = ( int((xyxy[0]+xyxy[2])/2), int((xyxy[1]+xyxy[3])/2))

                    # distance = aligned_depth_frame.get_distance(  xy_coord[0], xy_coord[1] )
                    distance = self.depth_np[xy_coord[1], xy_coord[0]]

                    cv2.putText(im0,
                                "{:.4f}".format(distance),
                                xy_coord, 
                                cv2.FONT_HERSHEY_SIMPLEX, 
                                0.8,
                                colors(c, True), 
                                1, 
                                cv2.LINE_AA
                    )                    

            #im0=im0[0,:,:,:]
            #print("changed im0 shpae : ",im0.shape)
            #im0=im0.transpose((1, 2, 0))
            #cv2.imshow("yolo", im0)
            print(s)
            #LOGGER.info(f"{s}")


            #cv2.imshow("yolo", im0) # 랙걸릴 땐 이것만 켜기.
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_np, alpha=0.03), cv2.COLORMAP_JET)
            images = np.hstack((im0, depth_colormap))#가로로 왼쪽 오른쪽 붙인 것.
            

            cv2.namedWindow('Align Example',  flags=cv2.WINDOW_AUTOSIZE)
            cv2.imshow('Align Example', images)

            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break



def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default=ROOT / 'yolov5s.pt', help='model path or triton URL')
    parser.add_argument('--source', type=str, default=ROOT / 'data/images', help='file/dir/URL/glob/screen/0(webcam)')
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
    parser.add_argument('--vid-stride', type=int, default=1, help='video frame-rate stride')
    opt = parser.parse_args()
    opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # expand
    print_args(vars(opt))
    return opt


def main(opt):

    rospy.init_node("yolov5", anonymous=True)
    detector = Yolov5Detector(**vars(opt))
    
    rospy.spin()
    


if __name__ == "__main__":
    opt = parse_opt()
    main(opt)