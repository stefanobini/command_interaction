#!/usr/bin/python3
# import cv2
import numpy as np
import os 
# import tensorflow as tf
from PIL import Image
from tensorflow.keras.models import load_model
import tensorflow.keras.backend as K
# from tensorflow.keras.preprocessing import image


# os.environ['CUDA_VISIBLE_DEVICES']=""
PATH_MODELS = os.path.join( os.path.dirname(os.path.realpath(__file__)), "..", "..", "..", "..", "models")

model_name_classification = os.path.join(PATH_MODELS, 'classificator/mobilenet_gesture.h5')
#classification_model = tf.saved_model.load(str(model_name_classification))
def contrastive_accuracy(y_true, y_pred):
    return  1.0-K.mean(K.max(abs(y_pred-y_true),axis=1))



class SlidingWindow:
    def __init__(self, window_size:int, element_size:int):
        self.window_size = window_size
        self.window = np.zeros((window_size,element_size))
        self.insert_idx = 0

    def put(self, element):
        self.window[self.insert_idx] = element
        self.insert_idx += 1
        self.insert_idx = 0 if self.insert_idx == self.window_size else self.insert_idx

    def get_max(self):
        avg_window = np.mean(self.window, axis=0)
        return np.argmax(avg_window), np.amax(avg_window)

    def get_window(self):
        return self.window
