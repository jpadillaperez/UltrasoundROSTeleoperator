from torch2trt import torch2trt, tensorrt_converter
import torch
import torch.nn.functional as F
import tensorrt as trt
import trt_pose.models
import json
import trt_pose.coco

# Load human pose JSON file
with open('/home/victorkawai/trt_pose/tasks/human_pose/human_pose.json', 'r') as f:
    human_pose = json.load(f)

# Convert COCO category to topology
topology = trt_pose.coco.coco_category_to_topology(human_pose)

# Get the number of parts and links
num_parts = len(human_pose['keypoints'])
num_links = len(human_pose['skeleton'])

# Load the model
model = trt_pose.models.resnet18_baseline_att(num_parts, 2 * num_links).cuda().eval()
MODEL_WEIGHTS = '/home/victorkawai/trt_pose/tasks/human_pose/resnet18_baseline_att_224x224_A_epoch_249.pth'
model.load_state_dict(torch.load(MODEL_WEIGHTS))

# # Define a custom converter for Conv2d
# @tensorrt_converter('torch.nn.Conv2d.forward')
# def convert_conv2d(ctx):
#     module = ctx.method_args[0]
#     input = ctx.method_args[1]
#     output = ctx.method_return

#     kernel = module.weight.detach().cpu().numpy()
#     bias = module.bias.detach().cpu().numpy() if module.bias is not None else None

#     # Debug prints to inspect the parameters
#     print("Converting Conv2d Layer:")
#     print("Input TRT: ", input._trt)
#     print("Output channels: ", module.out_channels)
#     print("Kernel shape: ", kernel.shape)
#     print("Bias shape: ", bias.shape if bias is not None else None)
#     print("Stride: ", module.stride)
#     print("Padding: ", module.padding)
#     print("Dilation: ", module.dilation)
#     print("Groups: ", module.groups)

#     # Create TensorRT layer
#     layer = ctx.network.add_convolution_nd(
#         input=input._trt,
#         num_output_maps=module.out_channels,
#         kernel_shape=kernel.shape[2:],
#         kernel=trt.Weights(kernel),
#         bias=trt.Weights(bias) if bias is not None else None
#     )

#     if layer is None:
#         raise RuntimeError("Failed to create convolution layer in TensorRT")

#     layer.stride_nd = tuple(module.stride)
#     layer.padding_nd = tuple(module.padding)
#     layer.dilation_nd = tuple(module.dilation)
#     layer.num_groups = module.groups
    
#     output._trt = layer.get_output(0)

# Define a custom converter for ConvTranspose2d
@tensorrt_converter('torch.nn.ConvTranspose2d.forward')
def convert_conv_transpose2d(ctx):
    module = ctx.method_args[0]
    input = ctx.method_args[1]
    output = ctx.method_return

    kernel = module.weight.detach().cpu().numpy()
    bias = module.bias.detach().cpu().numpy() if module.bias is not None else None

    # Debug prints to inspect the parameters
    print("Converting ConvTranspose2d Layer:")
    print("Input TRT: ", input._trt)
    print("Output channels: ", module.out_channels)
    print("Kernel shape: ", kernel.shape)
    print("Bias shape: ", bias.shape if bias is not None else None)
    print("Stride: ", module.stride)
    print("Padding: ", module.padding)
    print("Dilation: ", module.dilation)
    print("Groups: ", module.groups)

    # Create TensorRT layer
    layer = ctx.network.add_deconvolution_nd(
        input=input._trt,
        num_output_maps=module.out_channels,
        kernel_shape=kernel.shape[2:],
        kernel=trt.Weights(kernel),
        bias=trt.Weights(bias) if bias is not None else None
    )

    if layer is None:
        raise RuntimeError("Failed to create deconvolution layer in TensorRT")

    layer.stride_nd = tuple(module.stride)
    layer.padding_nd = tuple(module.padding)
    layer.dilation_nd = tuple(module.dilation)
    layer.num_groups = module.groups
    
    output._trt = layer.get_output(0)

# Define input dimensions
WIDTH = 224
HEIGHT = 224
data = torch.zeros((1, 3, HEIGHT, WIDTH)).cuda()

# Convert the model to TensorRT using torch2trt
try:
    model_trt = torch2trt(model, [data], fp16_mode=True, max_workspace_size=1<<25)
    print("Conversion successful!")
except Exception as e:
    print("Error during conversion: ", e)
