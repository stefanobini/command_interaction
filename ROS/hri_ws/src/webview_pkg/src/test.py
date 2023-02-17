import numpy as np

from utils import ImageStream


img_stream = ImageStream.instance()

while True:
    frame = img_stream.get_frame()
    if frame is None:
        print(frame)
    else:
        print(frame.shape)