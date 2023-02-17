import numpy as np



class ImageSource:
    '''Class defining the interface of a ImageSource object.

    The methods to implement are:
    
    - **\_\_init\_\_(self)**: constructor that takes as input the source settings (width, heigth, fps, ... )
    - **get_color_frame(self)**: returns a numpy rgb image 
    - **get_depth_frame(self)**: returns a numpy depth image 
    - **get_rgbd_frame(self)**: returns a numpy rgbd image 
    - **get_fov(self, mode)**: returns the FOV of the camera
    - **stop(self)**: this function close open streams
    '''

    def __init__(self):
        pass

    def get_color_frame(self):
        raise NotImplementedError
    
    def get_depth_frame(self):
        raise NotImplementedError
    
    def get_rgbd_frame(self):
        raise NotImplementedError

    def get_fov(self, mode="RGB"):
        raise NotImplementedError
    
    def stop(self):
        raise NotImplementedError

class Singleton(type):
    _instances = {}
    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]