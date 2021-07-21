#!/usr/bin/env python3
from charset_normalizer import detect
from cv_bridge import CvBridge
from one_stage_detector import OneStageDetector
# from demo_utils.ai.video.hand_detection import HandDetector
from demo_utils.gesture_recognition import SlidingWindow
from sensor_msgs.msg import Image
from std_msgs.msg import String
# from settings import demo_settings
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
import rospy
import time
import cv2
import numpy as np
from demo_utils.overlap_tracking_hand import CentroidTracker
import os
# https://github.com/robertanto/DemoFramework/invitations

os.environ['TF_GPU_ALLOCATOR'] = 'cuda_sqmalloc_async'

def convert(x):
    
    if x == 1:
        label = "Come here"
    elif x == 2:
        label = "Go"
    elif x == 3:
        label = "Start"
    elif x == 4:
        label = "Stop"
    elif x == 5:
        label = "Move To Right"
    elif x == 6:    
        label = "Move To Left"
    elif x == 7 or x == 8:    
        label = "Move Up"
    elif x == 9 or x == 10:    
        label = "Move Down"
    elif x == 11:    
        label = "Move Forward"
    elif x == 12:    
        label = "Move Backward"
    elif x == 13 or x == 0:    
        label = "NoGesture"
    print(label)
    return label


class Callback:

    def __init__(self, publisher, detector):
        self.bridge = CvBridge()
        self.detector = detector

        self.publisher = publisher
        self.hands_tracker = CentroidTracker(maxDisappeared=5)
        self.window_size = 6
        self.n_gestures = 14
        self.sliding_windows = dict()
        self.frame = 0
        self.timestamp = time.time()
        self.bridge = CvBridge()


    def __call__(self, msg):
        response = Detection2DArray()
        response.header.stamp = msg.header.stamp
        img = np.ndarray(shape=(msg.height, msg.width, 3), dtype=np.uint8, buffer=msg.data) # RAW image
        prevoius_timestamp = self.timestamp
        
        hands = self.detector.detect(img)

        self.timestamp = time.time()
        det_time = self.timestamp - prevoius_timestamp    # -> 0.13 s
        frame_rate = round(1 / (det_time), 2)
        print(frame_rate)
        ## NO SLIDING
        '''
        label_predicted = int(hands['label'])
        score_prediction = float(hands['confidence'])
        
        ## build classification message
        detection = Detection2D()
        detection.header.frame_id = convert(label_predicted)
        detection.bbox.size_x = hands['roi'][2] - hands['roi'][0]
        detection.bbox.size_y = hands['roi'][3] - hands['roi'][1]
        detection.bbox.center.x = (hands['roi'][2] + hands['roi'][0]) // 2
        detection.bbox.center.y = (hands['roi'][3] + hands['roi'][1]) // 2
        result = ObjectHypothesisWithPose()
        result.id = 0
        result.score = score_prediction
        detection.results.append(result)
        response.detections.append(detection)

        self.publisher.publish(response)
        '''
        
        
        hand_bboxes = list()

            ## compute bounding box coordinates -> hand['roi'] == [left, top, right, down]
            # width = hand['roi'][2] - hand['roi'][0] + x_offset
            # height = hand['roi'][3] - hand['roi'][1] + y_offset
        x_center = (hands['roi'][2] + hands['roi'][0]) // 2
        y_center = (hands['roi'][3] + hands['roi'][1]) // 2
        
        hand_bboxes.append(hands['roi'])
        
        centr_hands_in_frame, window_hands_in_frame = self.hands_tracker.update(rects=hand_bboxes)
        
        # width = hand['roi'][2] - hand['roi'][0] + x_offset
        # height = hand['roi'][3] - hand['roi'][1] + y_offset
        x_center = (hands['roi'][2] + hands['roi'][0]) // 2
        y_center = (hands['roi'][3] + hands['roi'][1]) // 2
        
        for (objectID, (centroid_x, centroid_y)) in centr_hands_in_frame.items():
            if centroid_x==x_center and centroid_y==y_center:
                ### enlarge bounding box
                #left = hand['roi'][0] - enlarge_x
                #top = hand['roi'][1] - enlarge_y
                #right = hand['roi'][2] + enlarge_x
                #bottom = hand['roi'][3] + enlarge_y
                probabilty_vector = np.zeros(14)
                label_predicted = int(hands['label'])
                score_prediction = float(hands['confidence'])
                probabilty_vector[label_predicted] = score_prediction
                

                ## add prediction to the sliding window
                if objectID not in self.sliding_windows:
                    self.sliding_windows[objectID] = SlidingWindow(window_size=self.window_size, element_size=self.n_gestures)
                    self.sliding_windows[objectID].put(probabilty_vector)
                else:
                    self.sliding_windows[objectID].put(probabilty_vector)
                
                ## apply majority voting on sliding window for gesture classification
                gesture, confidence = self.sliding_windows[objectID].get_max()
                gesture = convert(gesture)
                # print('Frame: {}\nHandID: {}\nSliding window: {}\nGesture: {}\nConfidence: {}\n'.format(self.frame, objectID, self.sliding_windows[objectID].get_window(), gesture, confidence))
                
                ## build classification message
                detection = Detection2D()
                detection.header.frame_id = gesture
                detection.bbox.size_x = window_hands_in_frame[objectID][0]
                detection.bbox.size_y = window_hands_in_frame[objectID][1]
                detection.bbox.center.x = centroid_x
                detection.bbox.center.y = centroid_y
                result = ObjectHypothesisWithPose()
                result.id = objectID
                result.score = confidence
                detection.results.append(result)
                ##
                #msg = self.bridge.cv2_to_imgmsg(img)
                detection.source_img =self.bridge.cv2_to_imgmsg(img)
                ##
                response.detections.append(detection)
                #print('Frame: {}\nobjectID: {}\nwidth: {}\nheight: {}\ncentroid_x: {}\ncentroid_y: {}\ngesture: {}\nconfidence: {}\n'.format(self.frame, type( str(objectID)), type(window_hands_in_frame[objectID][0]), type(window_hands_in_frame[objectID][1]), type(centroid_x), type(centroid_y), type(gesture), type('{:.2f}'.format(confidence))))           
        
        self.frame += 1
        
        self.publisher.publish(response)
        


class HandDetectorNode:


    def start(self):
        rospy.init_node('hand_gesture_recognition_node', anonymous=True)
        pub = rospy.Publisher("hand_gesture_recognition", Detection2DArray, queue_size=1)
        
        detector = OneStageDetector(0.4, None)
        # detector.compat.v1.summary()
        # classifier.compat.v1.summary()
        callback = Callback(pub, detector)
        print ('#################\n#               #\n# MODELS LOADED #\n#               #\n#################')
        sub = rospy.Subscriber("in_rgb", Image, callback)

        rospy.spin()


if __name__ == "__main__":
    try:
        node = HandDetectorNode()
        node.start()
    except rospy.ROSInterruptException:
        pass