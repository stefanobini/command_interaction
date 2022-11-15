#!/usr/bin/env python3

#Import necessary libraries
from flask import Flask, render_template, Response, stream_with_context, request
import cv2
import time
import numpy as np

from utils import ImageStream


#Initialize the Flask app
app = Flask(__name__)

print_var = "Running ..."

# Take image from 
# img_stream = ImageStream.instance()


"""The gen_frames() function enters a loop where it continuously returns frames from the camera as response chunks. The function asks the camera to provide a frame then it yields with this frame formatted as a response chunk with a content type of image/jpeg, as shown above."""
def gen_frames():
    img_stream = ImageStream.instance()
    while True:
        frame = img_stream.get_frame()
        if frame is None:
            break
        else:
            print_var = frame.shape
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')  # concat frame one by one and show result


def gen_text():
    yield "Running ..."


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


if __name__ == "__main__":
    app.run(debug=True, host='0.0.0.0', use_reloader=False)