#!/usr/bin/python

import numpy as np
from auvlib.data_tools import xtf_data
from auvlib.bathy_maps import base_draper
import sys
import os
import cv2
import argparse

def display_draping(args):

    waterfall_bins = args.waterfall_bins
    nbr_pings = args.nbr_pings
    
    xtf_pings = xtf_data.xtf_sss_ping.read_data(args.xtf_file) # read sss

    # images for displaying results
    meas_im = np.zeros((nbr_pings, 2*waterfall_bins))
    model_im = np.zeros((nbr_pings, 2*waterfall_bins))
    normals_im = np.zeros((nbr_pings, 2*waterfall_bins, 3))

    cv2.namedWindow('Model image', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Meas image', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Normal image', cv2.WINDOW_NORMAL)

    cv2.resizeWindow('Model image', 256, 1000)
    cv2.resizeWindow('Meas image', 256, 1000)
    cv2.resizeWindow('Normal image', 256, 1000)

    # read the results, e.g. produced by save_draping_result.py
    draping_results = base_draper.ping_draping_result.read_data(args.input)

    for (i, ping), (left, right) in zip(enumerate(xtf_pings[:nbr_pings]), draping_results[:nbr_pings]):
        model = np.concatenate([np.flip(left.time_bin_model_intensities), right.time_bin_model_intensities])
        normals = np.concatenate([np.flip(left.time_bin_normals, axis=0), right.time_bin_normals], axis=0)
        meas = np.concatenate([np.flip(base_draper.compute_bin_intensities(ping.port, waterfall_bins)),
                                       base_draper.compute_bin_intensities(ping.stbd, waterfall_bins)])
        meas_im[-i, :] = meas
        model_im[-i, :] = model
        if normals.shape[1] > 0:
            normals_im[-i, :, :] = .5*(normals + 1.)
            normals_im[-i, :, 2] = 0.

    cv2.imshow("Model image", model_im)
    cv2.imshow("Meas image", meas_im)
    cv2.imshow("Normal image", normals_im)
    cv2.waitKey()

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--bins', action="store", dest="waterfall_bins", type=int, default=256,
                        help='Number of time bins to use in waterfall image')
    parser.add_argument('--nbr_pings', action="store", dest="nbr_pings", type=int, default=2000,
                        help='Number of sidescan pings to plot')
    parser.add_argument('--xtf', action="store", dest="xtf_file", type=str, default="xtf_pings.cereal",
                        help='Pre-parsed .cereal file with xtf_pings')
    parser.add_argument('--input', '-i', action="store", dest="input", type=str, default="draping_results.cereal",
                        help='Input .cereal file containing ping_draping_result::ResultsT')
    args = parser.parse_args()

    display_draping(args)
