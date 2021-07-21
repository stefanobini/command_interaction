#!/usr/bin/env python3

from flask import Flask, render_template, Response, stream_with_context, request
import cv2
import json
import message_filters
import os
import rospy
import time
import threading
from threading import Lock

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String

from utils import ImageStream, TextStream


NODE_NAME = "webview_node"

app = Flask(__name__)   # Initialize the Flask app

imageStream = ImageStream()     # Initialize video stream
textStream = TextStream()       # Initialize text stream


"""The gen_frames() function enters a loop where it continuously returns frames from the camera as response chunks. The function asks the camera to provide a frame then it yields with this frame formatted as a response chunk with a content type of image/jpeg, as shown above."""
def gen_frames():
    while True:
        frame = imageStream.get_frame()
        if frame is None:
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')  # concat frame one by one and show result


def gen_text():
    text = textStream.get_text()
    yield text


"""Routes refer to URL patterns of an app (such as myapp.com/home or myapp.com/about). @app.route("/") is a Python decorator that Flask provides to assign URLs in our app to functions easily."""
@app.route('/')
def index():
    # return render_template('./webserver/templates/index.html')
    return render_template('index.html')


"""Update video container, with the images acquired by the cam and containing the bounding box of the hand with the detected gesture"""
@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


"""Update text bar, for detected commands"""
@app.route('/text_feed')
def text_feed():
    return Response(gen_text(), mimetype='text')



class ConsoleWebviewNode:
    '''ConsoleWebviewNode implements a ROS interface to show data on the Pepper's tablet through the web interface.
    The node subscribes to the following topics:
    
    - **sync_rgb**: raw images from the sync node
    - **rosout**: All the loggers publish here
    - **tracking_state**: all the data about the people in the scene
    The node does not publish on any topic.
    The available methods are:
    
    - **\_\_init\_\_(self)**: constructor
    - **start(self)**: starts the ros node instance
    '''
    def __init__(self, image, text, webserver):
        self.image_callback = image
        self.text_callback = text
        self.webserver = webserver


    def start(self):
        rospy.init_node(NODE_NAME, log_level=rospy.DEBUG)
        
        raw_image = message_filters.Subscriber('in_rgb', Image)
        gesture = message_filters.Subscriber('hand_gesture_recognition', Detection2DArray)
        ts = message_filters.TimeSynchronizer([raw_image, gesture], 100)  #50 
        ts.registerCallback(self.image_callback)

        sub = rospy.Subscriber("speech_command", String, self.text_callback)
        
        # threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False, threaded=True)).start()
        self.webserver.run(debug=True, host='0.0.0.0', use_reloader=False, threaded=True)
        # rospy.spin()


if __name__ == "__main__":
    
    try:
        console = ConsoleWebviewNode(image=imageStream, text=textStream, webserver=app)
        console.start()
    except rospy.ROSInterruptException:
        pass