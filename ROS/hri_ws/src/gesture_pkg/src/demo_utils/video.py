import sys
sys.path.append('..')

from . import ImageSource, Singleton
#from .. import Singleton
import numpy as np
import pyrealsense2 as rs
import cv2
from settings.io import io

class RealSenseImageSource(ImageSource, metaclass=Singleton):
    '''RealSenseImageSource implements ImageSource through the use of the Pyrealsense2 libraries.

    # Arguments
        width: int
            The width of the image - `default 640`
        height: int
            The height of the image - `default 480`
        fps: int
            The number of frames per second - `default 20`
        
    For more details refer to the Pyrealsense2 docs.
    '''

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
        while not loaded_correct:
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
            config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
             ## ADDED##
             

            self.pipeline.start(config)

            #ADDED

            self.colorizer = rs.colorizer()
            self.colorizer.set_option(rs.option.color_scheme, 0)
            try:
                self.pipeline.wait_for_frames() # (optional) get first frame to detect failures early
                loaded_correct = True
            except Exception as e:
                print(e) #TODO redirect error to logger node
                loaded_correct = False
                self.pipeline.stop()

    def get_color_frame(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        return np.asanyarray(color_frame.get_data())
    
    def get_depth_frame(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()       
        return np.asanyarray(self.colorizer.colorize(frames.get_depth_frame()).get_data())
        #return np.asanyarray(depth_frame.get_data())
    
    def get_rgbd_frame(self):
        frames = self.pipeline.wait_for_frames()
        align = rs.align(rs.stream.color)
        frames = align.process(frames)
        color_frame = frames.get_color_frame()
        color_frame = np.asanyarray(color_frame.get_data())
        depth_frame = frames.get_depth_frame()
        depth_frame = np.asanyarray(depth_frame.get_data())
        depth_frame = (depth_frame//256).astype(np.uint8)
        return np.dstack((color_frame, depth_frame))

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
        self.pipeline.stop()


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