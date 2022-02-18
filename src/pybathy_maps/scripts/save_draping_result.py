#!/usr/bin/python

import numpy as np
from auvlib.data_tools import std_data, xtf_data, xyz_data, csv_data
from auvlib.bathy_maps import mesh_map, base_draper, draw_map
import math
import sys
import os
import cv2
import argparse

def run_draping(args):

    mesh_res = args.mesh_res
    waterfall_bins = args.waterfall_bins
    sensor_offset = np.array(args.sensor_offset)
    sensor_yaw = args.sensor_yaw
    
    cloud = xyz_data.cloud.parse_file(args.xyz_file)
    height_map, bounds = mesh_map.height_map_from_dtm_cloud(cloud, mesh_res)

    if os.path.exists("mesh.npz"): # use cached mesh if it exists
        data = np.load("mesh.npz")
        V, F, bounds = data['V'], data['F'], data['bounds']
    else:
        V, F, bounds = mesh_map.mesh_from_dtm_cloud(cloud, mesh_res)
        np.savez("mesh.npz", V=V, F=F, bounds=bounds)

    xtf_pings = xtf_data.xtf_sss_ping.read_data(args.xtf_file) # read sss
    xtf_pings = xtf_data.correct_sensor_offset(xtf_pings, sensor_offset)

    sound_speeds = csv_data.csv_asvp_sound_speed.parse_file(args.asvp_file)

    # initialize a draper object that will accept sidescan pings
    draper = base_draper.BaseDraper(V, F, bounds, sound_speeds)
    draper.set_sidescan_yaw(sensor_yaw)
    draper.set_ray_tracing_enabled(False)

    # images for displaying results
    meas_im = np.zeros((2000, 2*waterfall_bins))
    model_im = np.zeros((2000, 2*waterfall_bins))
    normals_im = np.zeros((2000, 2*waterfall_bins, 3))

    cv2.namedWindow('Model image', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Meas image', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Normal image', cv2.WINDOW_NORMAL)

    cv2.resizeWindow('Model image', 256, 1000)
    cv2.resizeWindow('Meas image', 256, 1000)
    cv2.resizeWindow('Normal image', 256, 1000)

    # create a bathymetry height map for showing vehicle position
    d = draw_map.BathyMapImage(height_map, bounds)

    draping_results = [] # results list

    for i, ping in enumerate(xtf_pings):
        left, right = draper.project_ping(ping, waterfall_bins) # project
        draping_results.append((left, right)) # store result

        # the rest is basically just for visualizing the images
        model = np.concatenate([np.flip(left.time_bin_model_intensities), right.time_bin_model_intensities])
        normals = np.concatenate([np.flip(left.time_bin_normals, axis=0), right.time_bin_normals], axis=0)
        meas = np.concatenate([np.flip(base_draper.compute_bin_intensities(ping.port, waterfall_bins)),
                                       base_draper.compute_bin_intensities(ping.stbd, waterfall_bins)])
        meas_im[1:, :] = meas_im[:-1, :]
        meas_im[0, :] = meas
        model_im[1:, :] = model_im[:-1, :]
        model_im[0, :] = model
        normals_im[1:, :, :] = normals_im[:-1, :, :]
        if normals.shape[1] > 0:
            normals_im[0, :, :] = .5*(normals + 1.)
            normals_im[0, :, 2] = 0.
        if i % 10 == 0:
            d.draw_height_map(height_map) # draw the height map
            d.draw_blue_pose(ping.pos_, ping.heading_)
            d.blip()
            cv2.imshow("Model image", model_im)
            cv2.imshow("Meas image", meas_im)
            cv2.imshow("Normal image", normals_im)
            cv2.waitKey(1)

    base_draper.write_data(draping_results, args.output) # save results

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--mesh_res', action="store", dest="mesh_res", type=float, default=.5,
                        help='Resolution of produced mesh')
    parser.add_argument('--xyz', action="store", dest="xyz_file", type=str, default="KTH_Post_Deployment_AVG_WGS84UTM32N_RH200_50cm.xyz",
                        help='XYZ bathymetry file')
    parser.add_argument('--asvp', action="store", dest="asvp_file", type=str, default="KTH_PI_SVP_20180807_1251_573365N_0115014E_004.asvp",
                        help='ASVP sound speed file')
    parser.add_argument('--xtf', action="store", dest="xtf_file", type=str, default="xtf_pings.cereal",
                        help='Pre-parsed .cereal file with xtf_pings')
    parser.add_argument('--bins', action="store", dest="waterfall_bins", type=int, default=256,
                        help='Number of time bins to use in waterfall image')
    parser.add_argument('--sensor_yaw', action="store", dest="sensor_yaw", type=float, default=5.*math.pi/180.,
                        help='Yaw of the physical sidescan wrt vehicle')
    parser.add_argument('--sensor_offset', nargs='+', dest="sensor_offset", type=float, default=[2., -1.5, 0.],
                        help='Offset of the physical sensor wrt vehicle')
    parser.add_argument('--output', '-o', action="store", dest="output", type=str, default="draping_results.cereal",
                        help='Produce output file with ping_draping_result::ResultsT')
    args = parser.parse_args()

    run_draping(args)
