#!/usr/bin/env python

import cv2
import numpy as np
import time
import os

from sensor_msgs.msg import Image
from std_msgs.msg import String

from gesture_commands import GESTURE_COMMANDS


LANG = "eng"


class ImageStream(object):

    def __init__(self):
        self.image = cv2.imread("{}/static/assets/loading.png".format(os.path.dirname(os.path.abspath(__file__))))
        self.previous_label = None
        self.timestamp = time.time()
        self.count = 0
        #saves_path = "/home/felice/speech-command_interaction/ROS/speech_ws/src/gesture_pkg/src/saves"
        #out = cv2.VideoWriter('{}/filename.avi'.format(saves_path), cv2.VideoWriter_fourcc(*'MJPG'), 6, (320,320))


    def get_frame(self):
        # print("Video streaming: ON")
        return self.image


    def __call__(self, msg_img, msg_state):
        # print("Calling ImageStream callback")

        ## acquire image
        img = np.ndarray(shape=(msg_img.height, msg_img.width, 3), dtype=np.uint8, buffer=msg_img.data) # RAW image
        self.count += 1
        ## compute frame rate
        previous_timestamp = self.timestamp
        self.timestamp = time.time()
        frame_rate = round(1 / (self.timestamp - previous_timestamp), 2)
        
        (w_fps, h_fps), _ = cv2.getTextSize(str(frame_rate), cv2.FONT_HERSHEY_SIMPLEX, 1e-3 * 3 * 200, 2)
        x_fps, y_fps = 10, 10
        offset = 5
        #cv2.rectangle(img, (x_fps-offset, y_fps-offset), (x_fps+w_fps+offset, y_fps+h_fps+offset),  (0, 0, 0), 2)
        #cv2.putText(img, str(frame_rate), (x_fps,  y_fps+h_fps), cv2.FONT_HERSHEY_SIMPLEX, 1e-3 * 3 * 200, (0, 0, 0), 2)

        #response.header.stamp = msg.header.stamp
        #frame = bridge.imgmsg_to_cv2(msg)
        
        ## show detections
        for detection in msg_state.detections:

            label = int(detection.header.frame_id)
            if label == 13:
                continue

            right = int(detection.bbox.center.x + detection.bbox.size_x//2)
            bottom = int(detection.bbox.center.y + detection.bbox.size_y//2)
            left = int(detection.bbox.center.x - detection.bbox.size_x//2)
            top = int(detection.bbox.center.y - detection.bbox.size_y//2)
            cv2.rectangle(img, (left, top), (right, bottom),  (0, 255, 0), 2)
            #cv2.putText(img, '{} ({:.2f})'.format(detection.header.frame_id, detection.results[0].score), (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 1e-3 * 3 * 200, (0, 0, 255), 2)
            cv2.putText(img, '{}'.format(GESTURE_COMMANDS[label][LANG]), (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 1e-3 * 3 * 200, (0, 0, 255), 2)
        
        #out.write(img)
        if not label == 13 and self.count > 10:
            if self.previous_label != label or self.count > 20:
                self.count = 0
                self.previous_label = label
                message = String()
                message.data = GESTURE_COMMANDS[label][LANG]
        self.image = img
        # print(self.image.shape)
        """ret, buffer = cv2.imencode('.jpg', img)
        if ret:
            try:
                # publish image contanined in "buffer"
                self.image = buffer
            except Exception as e:
                print("WatchDog/TabletFault") """


class TextStream(object):

    def __init__(self):
        self.text = "System is running ..."
        self.new = True


    def get_text(self):
        self.new = False
        return self.text


    def new_text(self):
        return self.new


    def __call__(self, msg):
        self.text = msg.data
        self.new = True