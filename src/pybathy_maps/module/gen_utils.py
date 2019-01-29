import numpy as np

def clip_to_interval(height_patch, interval):
    current_mean = np.mean(height_patch[height_patch < 0.])
    for i in range(0, 10):
        diff = height_patch - current_mean
        if np.any(np.isnan(diff)):
            break
        in_interval = np.logical_and(diff < 0.5*interval, diff > -0.5*interval)
        if not np.any(interval):
            break
        current_mean = np.mean(height_patch[in_interval])
    height_patch = height_patch - (current_mean - 0.5*interval)
    height_patch = np.minimum(np.maximum(height_patch, 0.), interval)
    return height_patch
