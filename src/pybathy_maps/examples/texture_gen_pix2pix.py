#!/usr/bin/python

from auvlib.data_tools import std_data, gsf_data, utils
from auvlib.bathy_maps import mesh_map
import sys
import os
import numpy as np
import torch
from matplotlib import pyplot as plt
import cv2 # needed for resizing
from auvlib.bathy_maps.gen_utils import clip_to_interval

def predict_sidescan(network, image_input):

    image_input = image_input.transpose(-1, 0, 1) # we have to change the dimensions from width x height x channel (WHC) to channel x width x height (CWH)
    image_input = image_input[np.newaxis, :] # reshape([1] + image_input.shape)

    image_input = 2.*(1./255.*image_input.astype(np.float32))-1.
    #image_input = 1./255.*image_input.astype(np.float32)
    real_B = torch.from_numpy(image_input).cuda() # create the image tensor
    fake_A = network(real_B)  # G(A)

    image_numpy = fake_A[0].cpu().detach().float().numpy()  # convert it into a numpy array
    image_numpy = (np.transpose(image_numpy, (1, 2, 0)) + 1) / 2.0 * 255.0  # post-processing: tranpose and scaling
    image_numpy = image_numpy.astype(np.uint8)

    return image_numpy

def load_network(network_path):

    from models import networks # from the models directory of https://github.com/junyanz/pytorch-CycleGAN-and-pix2pix

    netG = networks.define_G(3, 3, 64, 'unet_256', 'batch', True, 'normal', 0.02, [0])

    if isinstance(netG, torch.nn.DataParallel):
        print("Netg is parallell instance")
        netG = netG.module
    state_dict = torch.load(network_path, map_location="cuda")
    if hasattr(state_dict, '_metadata'):
        del state_dict._metadata
    netG.load_state_dict(state_dict)

    return netG

def generate_sss_map(height_map, network_path):

    network = load_network(network_path)

    rows, cols = height_map.shape

    sss_gen_map = np.zeros((rows, cols, 3), dtype=np.uint8)

    interval = 2.
    height, width = 256, 256
    for i in xrange(0, rows-20, 20):
        for j in xrange(0, cols-20, 20):
            height_patch = height_map[i:i+20, j:j+20]
            height_patch = clip_to_interval(height_patch, interval)
            height_patch = cv2.resize(height_patch, (width, height), interpolation = cv2.INTER_CUBIC)
            height_patch = (255./interval*height_patch).astype(np.uint8)
            height_image = cv2.applyColorMap(height_patch, cv2.COLORMAP_JET)
            generated = predict_sidescan(network, height_image)
            generated = cv2.resize(generated, dsize=(20, 20), interpolation=cv2.INTER_LINEAR)
            sss_gen_map[i:i+20, j:j+20, :] = generated

    return sss_gen_map.transpose(1, 0, 2)

def generate_or_load_gen_sss_map(height_map, network_path):

    if os.path.exists("gen_sss_map.jpg"):
        gen_sss_map = cv2.imread("gen_sss_map.jpg")
    else:
        gen_sss_map = generate_sss_map(height_map, network_path)
        cv2.imwrite("gen_sss_map.jpg", gen_sss_map)
    return gen_sss_map

network_path = sys.argv[2] #"/home/nbore/Installs/pytorch-CycleGAN-and-pix2pix/datasets/checkpoints/pix2pix_sonar24/latest_net_G.pth"

gsf_pings = utils.parse_or_load_gsf(sys.argv[1])
mbes_pings = gsf_data.convert_pings(gsf_pings)
V, F, bounds = mesh_map.mesh_from_pings(mbes_pings, 0.5)

resolution = (bounds[1, 0] - bounds[0, 0])/1333.
print "Resolution: ", resolution
height_map, bounds = mesh_map.height_map_from_pings(mbes_pings, resolution)

sss_gen_map = generate_or_load_gen_sss_map(height_map, network_path)

R, G, B = sss_gen_map[:, :, 0], sss_gen_map[:, :, 1], sss_gen_map[:, :, 2]

mesh_map.show_textured_mesh(V, F, R, G, B, bounds)
