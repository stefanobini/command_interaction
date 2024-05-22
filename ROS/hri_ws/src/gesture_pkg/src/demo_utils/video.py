import sys
sys.path.append('..')

from . import ImageSource, Singleton
import numpy as np
import cv2
from settings.io import io


class OpenCVImageSource(ImageSource, metaclass=Singleton):
    def __init__(self, width=640, height=480, fps=6):
        '''Init method

        Initializes the realsense pipeline

        # Arguments
            width: width of the image. Int.
            height: height of the image. Int.
            fps: framerate of the camera. Int.
        '''
        loaded_correct = False
        self.width = width
        self.height = height
        self.stream = cv2.VideoCapture(io.camera.device)

        ret, img = self.stream.read()
    

    def get_color_frame(self):
        ret, img = self.stream.read()
        return img

    def get_fov(self, mode="RGB"):
        hfov, vfov = 0, 0
        if mode == "RGB":
            hfov = 64
            vfov = 41
        elif mode == "DEPTH":
            hfov = 86
            vfov = 57
        return hfov, vfov
    
    def stop(self):
        self.stream.release()