# SSDF Lane Segmentation

This is the package to predict lane segmentation from images.

# Installlation

## Requirements

Run:
```
pip install -r requirements.txt
```

# Current Supported Segmentation Models

## Mobilenet Unet in Pytorch

You can run from a launch file for UIT simulation (please see the launch file and override the arguments if need):
```bash
roslaunch lane_seg uit.launch
```

If you want to run lane segmentation without using PyTorch JIT:
```bash
roslaunch lane_seg uit.launch use_jit:=false model_path:=/path/to/model/weight
```