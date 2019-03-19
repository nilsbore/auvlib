#!/usr/bin/python

from auvlib.data_tools import std_data, gsf_data, xtf_data, csv_data, utils
from auvlib.bathy_maps import mesh_map, patch_draper, data_vis, sss_gen_sim, gen_utils, map_draper, draw_map
import sys
import os
import numpy as np
import math
from models import networks # from the models directory of https://github.com/junyanz/pytorch-CycleGAN-and-pix2pix
from auvlib.bathy_maps.gen_utils import clip_to_interval
import torch
import cv2

def create_mesh(path):

    gsf_pings = utils.parse_or_load_gsf(path)
    mbes_pings = gsf_data.convert_pings(gsf_pings)
    V, F, bounds = mesh_map.mesh_from_pings(mbes_pings, 0.5)
    height_map, bounds = mesh_map.height_map_from_pings(mbes_pings, 0.5)

    return V, F, height_map, bounds

def match_or_load_xtf(xtf_path, csv_path):

    if os.path.exists("matched_cache.cereal"):
        xtf_pings = xtf_data.xtf_sss_ping.read_data("matched_cache.cereal")
    else:
        xtf_pings = utils.parse_or_load_xtf(xtf_path)
        nav_entries = utils.parse_or_load_csv(csv_path)
        xtf_pings = csv_data.convert_matched_entries(xtf_pings, nav_entries)
        xtf_data.write_data(xtf_pings, "matched_cache.cereal")
    return xtf_pings

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

    #netG = networks.define_G(3, 3, 64, 'unet_256', 'batch', True, 'normal', 0.02, [0])
    netG = networks.define_G(1, 1, 64, 'unet_256', 'batch', True, 'normal', 0.02, [0])

    if isinstance(netG, torch.nn.DataParallel):
        print("Netg is parallell instance")
        netG = netG.module
    state_dict = torch.load(network_path, map_location="cuda")
    if hasattr(state_dict, '_metadata'):
        del state_dict._metadata
    netG.load_state_dict(state_dict)

    return netG

def prepare_model_window(model_window):

    model_window = cv2.resize(np.rot90(model_window), (256, 256), interpolation=cv2.INTER_NEAREST)
    model_window = (255.*np.minimum(np.maximum(model_window, 0.), 1.)).astype(np.uint8)
    model_window = model_window[:, :, np.newaxis]

    return np.fliplr(model_window)

def prepare_generated(generated, height):

    generated = np.rot90(generated, -1)

    if generated.shape[1] == 256 and generated.shape[0] == height:
        return generated

    generated = cv2.resize(generated, (256, height), interpolation=cv2.INTER_LINEAR)

    return generated


class SSSGenerator(object):

    def __init__(self, network_path):
        
        self.network = load_network(network_path)

    def gen_wf_callback(self, depth_window):

        #print "Got gen callback!"

        np.savez("temp_test.npz", depth_window=depth_window)
        interval = 2.
        height, width = 256, 256
        depth_left = np.fliplr(depth_window[:, :256])
        depth_right = depth_window[:, 256:]
        #depth_left = prepare_depth_window(depth_left)
        depth_left = prepare_model_window(depth_left)
        cv2.imshow("Gen callback", depth_left)
        cv2.waitKey(10)
        #depth_right = prepare_depth_window(depth_right)
        depth_right = prepare_model_window(depth_right)

        generated_left = np.fliplr(predict_sidescan(self.network, depth_left))
        generated_right = np.fliplr(predict_sidescan(self.network, depth_right))
        cv2.imshow("Gen callback gen", generated_left[:, :, 0])
        cv2.waitKey(10)

        generated_left = prepare_generated(generated_left[:, :, 0], depth_window.shape[0])
        generated_right = prepare_generated(generated_right[:, :, 0], depth_window.shape[0])

        generated = np.concatenate((np.fliplr(generated_left), generated_right), axis=1)
        generated = 1./255.*generated.astype(np.float64)

        return generated

sensor_yaw = 5.*math.pi/180.
sensor_offset = np.array([2., -1.5, 0.])
network_path = "/home/nbore/Installs/pytorch-CycleGAN-and-pix2pix/datasets/checkpoints/pix2pix_cos2/latest_net_G.pth"
sss_from_waterfall = True

V, F, height_map, bounds = create_mesh(sys.argv[1])

xtf_pings = match_or_load_xtf(sys.argv[2], sys.argv[3])
xtf_pings = xtf_data.correct_sensor_offset(xtf_pings, sensor_offset)
sound_speeds = csv_data.csv_asvp_sound_speed.parse_file(sys.argv[4])

viewer = sss_gen_sim.SSSGenSim(V, F, xtf_pings, bounds, sound_speeds, height_map)
viewer.set_sidescan_yaw(sensor_yaw)
viewer.set_ray_tracing_enabled(False)
viewer.set_sss_from_waterfall(sss_from_waterfall)
viewer.set_gen_window_height(64)

generator = SSSGenerator(network_path)
if sss_from_waterfall:
    viewer.set_gen_callback(generator.gen_wf_callback)
else:
    viewer.set_gen_callback(generator.gen_callback)

#viewer.show()
map_images = map_draper.sss_map_image.read_data("map_images_cache4.cereal")


#cv2.imshow("Sim image", sim_image)
#cv2.imshow("Model image", model_image)
#cv2.waitKey()

dataset_name = "prediction_results"

if not os.path.exists(dataset_name):
    os.makedirs(dataset_name)

def save_images(counter, model_image, sim_image, gt_image, pos):

    trackdir = os.path.join(dataset_name, str(counter+1))
    if not os.path.exists(trackdir):
        os.makedirs(trackdir)

    gt_image = np.minimum(np.maximum(gt_image, 0.), 1.)

    cv2.imwrite(os.path.join(trackdir, "1.png"), (255.*np.rot90(model_image, k=2)).astype(np.uint8))
    cv2.imwrite(os.path.join(trackdir, "2.png"), (255.*np.rot90(sim_image, k=2)).astype(np.uint8))
    cv2.imwrite(os.path.join(trackdir, "3.png"), (255.*np.rot90(gt_image, k=2)).astype(np.uint8))

    #im = draw_map.BathyMapImage(std_pings, 1000, 1000)
    im = draw_map.BathyMapImage(height_map, bounds)
    im.draw_height_map(height_map)
    im.draw_track(pos)
    im.rotate_crop_image(pos[0], pos[-1], 40.)
    im.write_image(os.path.join(trackdir, "q.png"))

counter = 0
for i, m in enumerate(map_images):
    print m.sss_ping_duration
    model_image = viewer.draw_model_waterfall(m.sss_waterfall_model, m.sss_ping_duration)
    sim_image = viewer.draw_sim_waterfall(m.sss_waterfall_model)

    if m.sss_waterfall_image.shape[0] > 1000:
        half = m.sss_waterfall_image.shape[0]/2
        save_images(counter, model_image[:half, :], sim_image[:half, :], m.sss_waterfall_image[:half, :], m.pos[1:2*half])
        counter += 1
        save_images(counter, model_image[half:, :], sim_image[half:, :], m.sss_waterfall_image[half:, :], m.pos[2*half:])
    else:
        save_images(counter, model_image, sim_image, m.sss_waterfall_image, m.pos[1:])

    counter += 1


