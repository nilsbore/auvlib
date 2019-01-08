#!/usr/bin/python

import matplotlib.pyplot as plt
from auvlib.bathy_maps import patch_draper, data_vis
from numpy import sign
import numpy as np
from copy import deepcopy
import sys

patch_views = patch_draper.sss_patch_views.read_data(sys.argv[1])
#patch_views.append(patch_views[0])

print "Number of patches :", len(patch_views)

data_vis.plot_patch_views(patch_views)
