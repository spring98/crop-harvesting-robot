
## YOLOv5 by Ultralytics, GPL-3.0 license
"""
김보겸이 짰던 파일

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
                                         yolov5s.mlmodel            # CoreML (macOS-only)
                                         yolov5s_saved_model        # TensorFlow SavedModel
                                         yolov5s.pb                 # TensorFlow GraphDef
                                         yolov5s.tflite             # TensorFlow Lite
                                         yolov5s_edgetpu.tflite     # TensorFlow Edge TPU
"""

import os
import platform
import sys
import numpy as np
from pathlib import Path

import torch
import torch.backends.cudnn as cudnn

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
sys.path.append(str(FILE.parents[1])+'/yolov5_greencamp')  # yolov5 서브모듈 패스 넣어주기.
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

print(sys.path)

from models.common import DetectMultiBackend
from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadScreenshots, LoadStreams
from utils.general import (LOGGER, Profile, check_file, check_img_size, check_imshow, check_requirements, colorstr, cv2,
                           increment_path, non_max_suppression, print_args, scale_boxes, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, smart_inference_mode


weights = 'yolov5s.pt'
sourceO = 0  # file/dir/URL/glob, 0 for webcam // 사용 x depth camer 연결하기 때문.
data = ROOT / 'data/coco128.yaml'  # dataset.yaml path
imgszO = (640, 640)  # inference size (height, width)
conf_thres = 0.1  # confidence threshold
iou_thres = 0.45  # NMS IOU threshold
max_det = 1000  # maximum detections per image
deviceO = ''
view_img = False  # show results
save_txt = False  # save results to *.txt
save_conf = False  # save confidences in --save-txt labels
save_crop = False  # save cropped prediction boxes
nosave = False  # do not save images/videos
classes = None  # filter by class: --class 0, or --class 0 2 3
agnostic_nms = False  # class-agnostic NMS
augment = False  # augmented inference
visualize = False  # visualize features
update = False  # update all models
project = ROOT / 'runs/detect'  # save results to project/name
name = 'exp'  # save results to project/name
exist_ok = False  # existing project/name ok, do not increment
line_thickness = 3  # bounding box thickness (pixels)
hide_labels = False  # hide labels
hide_conf = False  # hide confidences
half = False  # use FP16 half-precision inference
dnn = False  # use OpenCV DNN for ONNX inference


def run():
    source = str(sourceO)
    webcam = source.isnumeric()

    # Load model
    device = select_device(deviceO)
    model = DetectMultiBackend(weights, device=device, dnn=dnn, data=data, fp16=half)
    stride, names, pt = model.stride, model.names, model.pt
    imgsz = check_img_size(imgszO, s=stride)  # check image size

    # Dataloader
    view_img = check_imshow()
    cudnn.benchmark = True  # set True to speed up constant image size inference
    dataset = LoadStreams(source, img_size=imgsz, stride=stride, auto=pt)
    bs = len(dataset)  # batch_size

    # Run inference
    model.warmup(imgsz=(1 if pt else bs, 3, *imgsz))  # warmup
    seen, windows, dt = 0, [], [0.0, 0.0, 0.0]
    for path, im, im0s, vid_cap, s in dataset:
        t1 = time_sync()
        im = torch.from_numpy(im).to(device)
        im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
        im /= 255  # 0 - 255 to 0.0 - 1.0
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim
        t2 = time_sync()
        dt[0] += t2 - t1

        pred = model(im, augment=False)
        t3 = time_sync()
        dt[1] += t3 - t2

        # NMS
        pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)
        dt[2] += time_sync() - t3

        # Process predictions
        for i, det in enumerate(pred):  # per image
            seen += 1
            if webcam:  # batch_size >= 1
                p, im0, frame = path[i], im0s[i].copy(), dataset.count
                s += f'{i}: '
            else:
                p, im0, frame = path, im0s.copy(), getattr(dataset, 'frame', 0)

            p = Path(p)  # to Path
            s += '%gx%g ' % im.shape[2:]  # print string
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            imc = im0.copy() if save_crop else im0  # for save_crop
            annotator = Annotator(im0, line_width=line_thickness, example=str(names))
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

                # Write results
                for *xyxy, conf, cls in reversed(det):
                    xmin = (xyxy[0] + xyxy[2]) / 2
                    ymin = (xyxy[1] + xyxy[3]) / 2

                    if view_img:  # Add bbox to image
                        c = int(cls)  # integer class
                        label = None if hide_labels else (names[c] if hide_conf else f'{names[c]} {conf:.2f}')
                        annotator.box_label(xyxy, label, color=colors(c, True))
                        origin_coord = f'({xmin}, {ymin})'
                        convert_x = ((xmin - 320) / 320) * 0.45
                        convert_y = ((480 - ymin) / 480) * 0.5

                        x = '{:.4f}'.format(convert_x)
                        y = '{:.4f}'.format(convert_y)

                        convert_coord = f'({x}, {y})'

                        cv2.circle(imc, (int(xmin), int(ymin)), 3, colors(c, True), -1, cv2.LINE_AA)
                        cv2.putText(imc, origin_coord, (int(xmin), int(ymin)), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                    colors(c, True), 1, cv2.LINE_AA)
                        cv2.putText(imc, convert_coord, (int(xmin), int(ymin) + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                    (67, 93, 255), 2, cv2.LINE_AA)

            # Stream results
            im0 = annotator.result()
            if view_img:
                if platform.system() == 'Linux' and p not in windows:
                    windows.append(p)
                    cv2.namedWindow(str(p), cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)  # allow window resize (Linux)
                    cv2.resizeWindow(str(p), im0.shape[1], im0.shape[0])
                cv2.imshow(str(p), im0)
                cv2.waitKey(1)  # 1 millisecond

        # Print time (inference-only)
        LOGGER.info(f'{s}Done. ({t3 - t2:.3f}s)')

    # Print results
    t = tuple(x / seen * 1E3 for x in dt)  # speeds per image
    LOGGER.info(f'Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS per image at shape {(1, 3, *imgsz)}' % t)

    if update:
        strip_optimizer(weights)  # update model (to fix SourceChangeWarning)


if __name__ == "__main__":
    check_requirements(exclude=('tensorboard', 'thop'))
    FILE = Path(__file__).resolve()
    ROOT = FILE.parents[0]  # YOLOv5 root directory
    print("YOLOV5 루트: ",ROOT)

    with torch.no_grad():
        run()
