import cv2
import numpy as np
import os 
import torch
from PIL import Image
from torchvision.models.detection.ssdlite import ssdlite320_mobilenet_v3_large as mobilenetv3ssd
from torchvision.transforms import functional as F
from torchvision.transforms import transforms as T, InterpolationMode
from commands import GESTURES
#import onnx
#import onnxruntime
#from onnxruntime.quantization import quantize_dynamic, QuantType

#import torch

# os.environ['CUDA_VISIBLE_DEVICES']=""
PATH_MODELS = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..",  "models")

#TORCH_PATH = os.path.join(PATH_MODELS, "15ep_2022_10_20_09_29_2th_finetuning_model.pt")
TORCH_PATH = os.path.join(PATH_MODELS, "demo7", "rgb.pt")
#TORCH_PATH = os.path.join(PATH_MODELS, "mobilenetv3_rgb_fp16.pt")
#TORCH_PATH = os.path.join(PATH_MODELS, "mobilenetv3_rgb_int8.pt")
# WEIGHTS_PATH = os.path.join(PATH_MODELS, "10ep_2022_10_20_13_45_2th_finetuning_model_depth.pt") #depth
ONNX_PATH = os.path.join(PATH_MODELS, "mobilenetv3_rgb.onnx")
ONNX_INT8_PATH = os.path.join(PATH_MODELS, "mobilenetv3_rgb_int8.onnx")
# WEIGHTS_PATH = os.path.join(PATH_MODELS, "mobilenetv3_rgb-sim.onnx")
PRETRAINED_WEIGHT = os.path.join(PATH_MODELS, "mobilenet_v3_large-8738ca79.pth")

MAX_BBOX_DETECTABLE = 1
# detector = os.path.join(PATH_MODELS, "detector/trained-inference-graphs_2m/output_inference_graph_v1.pb/saved_model")


def reframe_box_masks_to_image_masks(box_masks, boxes, image_height, image_width, resize_method='bilinear'):
  """Transforms the box masks back to full image masks.
  Embeds masks in bounding boxes of larger masks whose shapes correspond to
  image shape.
  Args:
    box_masks: A tensor of size [num_masks, mask_height, mask_width].
    boxes: A tf.float32 tensor of size [num_masks, 4] containing the box
           corners. Row i contains [ymin, xmin, ymax, xmax] of the box
           corresponding to mask i. Note that the box corners are in
           normalized coordinates.
    image_height: Image height. The output mask will have the same height as
                  the image height.
    image_width: Image width. The output mask will have the same width as the
                 image width.
    resize_method: The resize method, either 'bilinear' or 'nearest'. Note that
      'bilinear' is only respected if box_masks is a float.
  Returns:
    A tensor of size [num_masks, image_height, image_width] with the same dtype
    as `box_masks`.
  """
  resize_method = 'nearest' if box_masks.dtype == torch.uint8 else resize_method    # before box_masks.dtype == tf.uint8


class OneStageDetector:
    '''FaceDetector recognized all the faces in an image.

    Only the images which area is larger than the threshold are returned

    # Arguments
        conf_thresh: float
            The minimum confidence to recognize a face - `default 0.3`
        size_threhsold: float 
            The minimum area for a face to be published - `default None`
        
    For more details refer to opencv docs.
    '''

    def __init__(self, conf_thresh=0.3, size_thresh=None, n_gestures=len(GESTURES)):
        self.device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
        self.n_gestures = n_gestures
        #""" PyTorch model
        self.net = mobilenetv3ssd(pretrained_backbone=True, num_classes=self.n_gestures)
        #self.weights = torch.load(WEIGHTS_PATH)
        self.net.load_state_dict(torch.load(TORCH_PATH, map_location=self.device)["model_state_dict"])
        self.net.to(self.device)
        # self.net = torch.jit.optimize_for_inference(torch.jit.script(self.net.eval()))
        self.net.eval()
        # """
        """ ONNX Model
        # providers = ['TensorrtExecutionProvider', 'CUDAExecutionProvider', 'CPUExecutionProvider']
        providers = ['CPUExecutionProvider']
        self.ort_session = onnxruntime.InferenceSession(ONNX_PATH, providers=providers)
        #self.ort_session = onnxruntime.InferenceSession(ONNX_INT8_PATH, providers=providers)
        #"""

        self.confidence_threshold = conf_thresh
        self.size_threshold = size_thresh
    
    def crop_image_to_square(self, image):

        height = image.shape[0]
        width = image.shape[1]
        crop_size = min(height, width)
        new_height = min(height, crop_size)
        new_width = min(width, crop_size)
        offset_height = max(height - crop_size, 0)
        offset_width = max(width - crop_size, 0)
        #GET THE TOP AND LEFT COORDINATES OF A CENTRAL CROP
        top = int(offset_height/2)
        left = int(offset_width/2)
        image_arr = image[top : top + crop_size, left : left + crop_size]

        return image_arr

    def onnx_cpu_detect(self, image):
        """
        image:  numpy.float32
            The shape should be (Batch, Channel, Width, Height) 
        """
        # image = onnxruntime.OrtValue.ortvalue_from_numpy(image)
        image = np.expand_dims(image, axis=0)   # add batch dimension
        outputs = self.ort_session.run(
            ["boxes", "scores", "labels"],
            {"input": image},
        )
        return self.onnx_cpu_post_processing(boxes=outputs[0], scores=outputs[1], labels=outputs[2])

    def onnx_cpu_post_processing(self, boxes, scores, labels):
        print(np.amax(scores))
        pred_detections = np.count_nonzero(scores > self.confidence_threshold)
        if pred_detections > 0:
            boxes = boxes[:pred_detections]
            scores = scores[:pred_detections]
            labels = labels[:pred_detections]
            index_argmin = np.argmin(labels)  #check if a valid gesture is detected
            if  index_argmin != self.n_gestures:
                #If valid gesture is detected, we take the index of valid gesture [1,12] with higher scores
                index_valid_gesture = np.argmax((np.where(labels<self.n_gestures, scores, 0.)))
                boxes = boxes[index_valid_gesture]
                scores = scores[index_valid_gesture]
                labels = labels[index_valid_gesture]
            #if argmin == 13, only no gesture are detected
            else:
                boxes = boxes[:MAX_BBOX_DETECTABLE]
                scores = scores[:MAX_BBOX_DETECTABLE]
                labels = labels[:MAX_BBOX_DETECTABLE]

        #no prediction, skip frame e set "no-gesture"
        else:
            '''
            boxes = boxes[:MAX_BBOX_DETECTABLE]
            scores = scores[:MAX_BBOX_DETECTABLE]
            labels = labels[:MAX_BBOX_DETECTABLE]
            '''
            #print("NO GESTURE DETECTED")
            return {
                    'roi': [0.,0.,0.,0.],
                    'type': 'hand',
                    'label': self.n_gestures,
                    'confidence' : 0.
                    }

        hand_results = {'roi': boxes.tolist(),
                        'type': 'hand',
                        'label': float(labels),
                        'confidence' : float(scores)
                        }
        return hand_results

    def onnx_gpu_detect(self, image):
        """
        image:  numpy.float32
            The shape should be (Batch, Channel, Width, Height) 
        """
        # image = onnxruntime.OrtValue.ortvalue_from_numpy(image)
        image = np.expand_dims(image, axis=0)   # add batch dimension
        io_binding = self.ort_session.io_binding()
        io_binding.bind_cpu_input("input", image)
        io_binding.bind_output(["boxes", "scores", "labels"], "cuda")
        self.ort_session.run_with_iobinding(io_binding)
        outputs = io_binding.get_outputs()
        return self.onnx_post_processing(boxes=outputs[0], scores=outputs[1], labels=outputs[2])

    def onnx_gpu_post_processing(self, boxes, scores, labels):
        pred_detections = np.count_nonzero(scores > self.confidence_threshold)
        if pred_detections > 0:
            boxes = boxes[:pred_detections]
            scores = scores[:pred_detections]
            labels = labels[:pred_detections]
            index_argmin = np.argmin(labels)  #check if a valid gesture is detected
            if  index_argmin != self.n_gestures:
                #If valid gesture is detected, we take the index of valid gesture [1,12] with higher scores
                index_valid_gesture = np.argmax((np.where(labels<self.n_gestures, scores, 0.)))
                boxes = boxes[index_valid_gesture]
                scores = scores[index_valid_gesture]
                labels = labels[index_valid_gesture]
            #if argmin == 13, only no gesture are detected
            else:
                boxes = boxes[:MAX_BBOX_DETECTABLE]
                scores = scores[:MAX_BBOX_DETECTABLE]
                labels = labels[:MAX_BBOX_DETECTABLE]

        #no prediction, skip frame e set "no-gesture"
        else:
            '''
            boxes = boxes[:MAX_BBOX_DETECTABLE]
            scores = scores[:MAX_BBOX_DETECTABLE]
            labels = labels[:MAX_BBOX_DETECTABLE]
            '''
            #print("NO GESTURE DETECTED")
            return {
                    'roi': [0.,0.,0.,0.],
                    'type': 'hand',
                    'label': self.n_gestures,
                    'confidence' : 0.
                    }

        hand_results = {'roi': boxes.tolist(),
                        'type': 'hand',
                        'label': float(labels),
                        'confidence' : float(scores)
                        }
        return hand_results

    def detect(self, image):
        #assert self.device == torch.device('cuda')

        #image = self.crop_image_to_square(image)
        #image = F.to_tensor(np.array(image))
        #image = torch.tensor(image, device=self.device)
        '''
        red = image[2,:,:]
        green = image[1,:,:]
        blue = image[0,:,:]
        image = torch.stack([red, green, blue])
        #'''

        #image = F.resize(image, [320, 320], interpolation=InterpolationMode.BILINEAR)
        #image = list(image.to(self.device))
        image = [torch.tensor(image, device=self.device)]
        #'''
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        #'''
        #model_time = time.time()
        with torch.no_grad():
            outputs = self.net(image)
        return self.post_processing(outputs=outputs)
        
    def post_processing(self, outputs):
        outputs = outputs[0]     # take only one hand (the one with higher confidence)
        pred_detections = torch.count_nonzero(outputs["scores"] > self.confidence_threshold).item()
        if pred_detections > 0:
            outputs = self.reduce_size_tensor(outputs, pred_detections)
            index_argmin = torch.argmin(outputs["labels"]).item()  #check if a valid gesture is detected
            if  index_argmin != self.n_gestures-1:
                #If valid gesture is detected, we take the index of valid gesture [1,12] with higher scores
                index_valid_gesture = torch.argmax((torch.where(outputs["labels"]< self.n_gestures-1, outputs["scores"], torch.tensor(0., dtype=torch.float32, device=self.device))))
                outputs = {"boxes":outputs["boxes"][index_valid_gesture], "labels": outputs["labels"][index_valid_gesture], "scores": outputs["scores"][index_valid_gesture]}
            #if argmin == 13, only no gesture are detected
            else:
                outputs = self.reduce_size_tensor(outputs, MAX_BBOX_DETECTABLE) #take the no gesture with higher score

        #no prediction, skip frame e set "no-gesture"
        else:
            outputs = self.reduce_size_tensor(outputs, MAX_BBOX_DETECTABLE)
            #print("NO GESTURE DETECTED")
            return {
                            'roi': [0.,0.,0.,0.],
                            'type': 'hand',
                            'label': self.n_gestures-1,
                            'confidence' : 0.,
                            # 'rects': rects
                        }

        out_label_for_classification = float(outputs["labels"].item())
        out_bbox_for_classification = outputs["boxes"].tolist()
        out_score_for_classification = float(outputs["scores"].item())
        hand_results = {
                            'roi': out_bbox_for_classification,
                            'type': 'hand',
                            'label': out_label_for_classification,
                            'confidence' : out_score_for_classification,
                            # 'rects': rects
                        }
        return hand_results


    def reduce_size_tensor (self, tensor, size):
        return {k: v[:size] for k, v in tensor.items()}

