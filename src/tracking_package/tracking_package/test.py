from ultralytics import YOLO
import cv2
import cvzone
import time
import math
import numpy as np
from tracking_package.sort.sort import Sort

FILE_PATH = ""
classNames = ["trash"]

tracker = Sort(max_age=20, min_hits=3, iou_threshold = 0.3)
cap = cv2.VideoCapture(6)
model = YOLO(r"/home/hoonji/embed/src/tracking_package/tracking_package/best.pt")

while True:
    _, img = cap.read()
    if not _:
        break
    results = model(img, stream=True)
    detections = np.empty((0,5))

    for r in results:
        boxes = r.boxes
        for box in boxes:
            x1,y1,x2,y2 = box.xyxy[0]
            x1,y1,x2,y2 = int(x1),int(y1),int(x2),int(y2)

            #classname
            cls = int(box.cls[0])

            #confodence score
            conf = math.ceil(box.conf[0]*100)/100
            if conf > 0.5:
                cvzone.putTextRect(img, f'{classNames[cls]}', (x2, y2), scale=1, thickness=1, colorR=(0, 0, 255))

                currentArray = np.array([x1,y1,x2,y2,conf])
                detections = np.vstack((detections, currentArray))
    
    resultTracker = tracker.update(detections)

    for res in resultTracker:
        x1,y1,x2,y2,id = res
        x1,y1,x2,y2,id = int(x1),int(y1),int(x2),int(y2),int(id)
        w, h = x2-x1, y2-y1

        cvzone.putTextRect(img, f'{classNames[cls]}', (x2, y2), scale=1, thickness=1, colorR=(0, 0, 255))

    cv2.imshow('Image', img)
    if cv2.waitKey(1) == ord('q'):
        break