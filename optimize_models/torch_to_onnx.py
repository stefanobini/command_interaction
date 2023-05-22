import os
import io
import numpy as np

import torch
from torch import nn
import torch.utils.model_zoo as model_zoo
import torch.onnx

from torchvision.models.detection.ssdlite import ssdlite320_mobilenet_v3_large as mobilenetv3ssd


NUM_CLASSES = 14
MODEL_NAME = "mobilenetv3_rgb_fp32"
IN_FOLDER = os.path.join("models", "torch")
OUT_FOLDER = os.path.join("models", "onnx")
in_path = os.path.join(IN_FOLDER, MODEL_NAME) + ".pt"
out_path = os.path.join(OUT_FOLDER, MODEL_NAME+"_ssd") + ".onnx"


# Create model
torch_model = mobilenetv3ssd(pretrained_backbone = True, num_classes = NUM_CLASSES)

# Initialize model with the pretrained weights
device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
torch_model.load_state_dict(torch.load(in_path, map_location=device)['state_dict'])

# set the model to inference mode
torch_model.eval()

# Input to the model
x = torch.randn(1, 3, 320, 320, requires_grad=True)
torch_out = torch_model(x)

# Export the model
torch.onnx.export(torch_model,               # model being run
                  x,                         # model input (or a tuple for multiple inputs)
                  out_path,   # where to save the model (can be a file or file-like object)
                  export_params=True,        # store the trained parameter weights inside the model file
                  opset_version=11,          # the ONNX version to export the model to
                  do_constant_folding=True,  # whether to execute constant folding for optimization
                  input_names = ["input"],   # the model's input names
                  output_names = ["boxes", "scores", "labels"], # the model's output names
                  )
