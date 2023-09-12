import os
import torch
from torchvision.models.detection.ssdlite import ssdlite320_mobilenet_v3_large as mobilenetv3ssd


def model_size(model):
    param_size = 0
    for param in model.parameters():
        param_size += param.nelement() * param.element_size()
    buffer_size = 0
    for buffer in model.buffers():
        buffer_size += buffer.nelement() * buffer.element_size()

    size_all_mb = (param_size + buffer_size) / 1024**2
    return size_all_mb


PATH_MODELS = os.path.join("models", "torch")
FP32_PATH = os.path.join(PATH_MODELS, "mobilenetv3_rgb.pt")
FP16_PATH = os.path.join(PATH_MODELS, "mobilenetv3_rgb_fp16.pt")
#AMP_PATH = os.path.join(PATH_MODELS, "mobilenetv3_rgb_amp.pt")
INT8_PATH = os.path.join(PATH_MODELS, "mobilenetv3_rgb_int8.pt")

device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')

model_fp32 = mobilenetv3ssd(pretrained_backbone=True, num_classes=14)
model_fp32.load_state_dict(torch.load(FP32_PATH, map_location=device)["model_state_dict"])
model_fp16 = model_fp32.half()
#model_amp = model_fp32.amp()

size_fp32 = model_size(model=model_fp32)
size_fp16 = model_size(model=model_fp16)
#size_amp = model_size(model=model_amp)

'''Quantizzation'''
model_fp32.eval()
model_fp32.qconfig = torch.ao.quantization.get_default_qconfig('x86')

# Operators fusion
model_fp32_fused = torch.ao.quantization.fuse_modules(model_fp32, [['conv', 'relu']])
model_fp32_fused = model_fp32

# Insert observers in the models
model_fp32_prepared = torch.ao.quantization.prepare(model_fp32_fused)

# Quantization
input_fp32 = torch.randn(1,3,320,320)
#model_fp32_prepared(input_fp32)
model_int8 = torch.ao.quantization.convert(model_fp32_prepared)
#model_int8(input_fp32)
size_int8 = model_size(model=model_int8)

torch.save(model_fp16.state_dict(), FP16_PATH)
#torch.save(model_amp.state_dict(), AMP_PATH)
torch.save(model_int8.state_dict(), INT8_PATH)
print("Original model size:\t{}\nFloat16 model size:\t{}\nInt8 model size:\t{}".format(size_fp32, size_fp16, size_int8))