import os
import numpy as np


def filepath_to_filename(filepath):
    return os.path.basename(os.path.normpath(filepath))


def stack_hits_xyz_to_pos(data):
    """Given a sss_meas_data object, return the XYZ hits stacked
    into a numpy array"""
    return np.stack([
        data.sss_waterfall_hits_X, data.sss_waterfall_hits_Y,
        data.sss_waterfall_hits_Z
    ],
                    axis=-1)


def normalize_waterfall_image(waterfall_image, a_max=3):
    """Given a waterfall image (a 2D numpy array), divide each column
    by column mean and clip values between (0, a_max) for visualization."""
    waterfall_image = waterfall_image.copy()
    col_mean = waterfall_image.mean(axis=0)
    waterfall_image = np.divide(waterfall_image,
                                col_mean,
                                where=[col_mean != 0.])
    clipped_image = np.clip(waterfall_image, a_min=0, a_max=a_max)
    return clipped_image


def update_highlight(highlight, pixel=([], []), color=None):
    """Update the data and color of a Line2D object"""
    highlight.set_data(pixel[1], pixel[0])
    if color is not None:
        highlight.set_color(color)
    highlight.figure.canvas.draw_idle()
