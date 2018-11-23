#!/usr/bin/python

import matplotlib.pyplot as plt
from pybathy_maps import draping_viewer

patch_views = draping_viewer.sss_patch_views.read_data("patch_views.cereal")

print "Number of patches :", len(patch_views)

max_patches = max(len(p.sss_views) for p in patch_views)

f, axarr = plt.subplots(len(patch_views), max_patches+1)

for i, p in enumerate(patch_views):

    axarr[i, 0].imshow(p.patch_height)
    for j, im in enumerate(p.sss_views):
        axarr[i, j+1].imshow(im, vmin=0., vmax=1.)

    for j in range(len(p.sss_views), max_patches):
        axarr[i, j+1].axis('off')

plt.show()
