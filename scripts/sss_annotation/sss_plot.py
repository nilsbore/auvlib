import os
from collections import OrderedDict, defaultdict
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import KDTree
from auvlib.bathy_maps.map_draper import sss_meas_data

import sss_annotation.utils as utils


class SSSAnnotationPlot():
    def __init__(self,
                 filepath,
                 overlapping_filepaths,
                 figsize,
                 neighbour_thresh=.5,
                 normalize_image=True):
        self.neighbour_thresh = neighbour_thresh
        self.filepath = filepath
        self.normalize_image = normalize_image

        self.data = SSSPlotData(filepath, figsize, normalize_image)
        self.data.register_figure_callback(self.find_corr_pixels_onclick)

        self.num_overlapping_data = len(overlapping_filepaths)
        self.overlapping_data = self.plot_overlapping_data(
            overlapping_filepaths, figsize, normalize_image)
        self.annotations = defaultdict(dict)

        self.target_pos = None
        self.target_coord = None  # (ping_nbr, col_nbr)
        self.colors = {'target_pixel': 'red', 'annotation': 'yellow'}

    def plot_overlapping_data(self, overlapping_filepaths, figsize,
                              normalize_image):
        """Plot all waterfall images in overlapping_filepaths and register
        the annotate_onclick callback to each figure.
        Return a dictionary with key = filepath and value = SSSPlotData object"""
        overlapping_data = OrderedDict()
        for filepath in overlapping_filepaths:
            data = SSSPlotData(filepath, figsize, normalize_image)
            data.register_figure_callback(self.annotate_onclick)
            overlapping_data[filepath] = data
        return overlapping_data

    def annotate_onclick(self, event):
        if not event.dblclick or self.target_coord is None:
            return
        try:
            coord = (int(event.ydata), int(event.xdata))
        except TypeError as e:
            return
        clicked_axis = event.inaxes
        for k, v in self.overlapping_data.items():
            if v.ax == clicked_axis:
                name = v.filename
                self.update_annotation(name, coord)
                utils.update_highlight(v.highlight, coord,
                                       self.colors['annotation'])

    def update_annotation(self, corr_filename, corr_pixel):
        """Add the new annotation to the annotations field. Each annotation
        is a dictionary entry with key=target coordinate and value=a dictionary
        with k=corr_filename, v=corr_pixel"""
        self.annotations[self.target_coord][corr_filename] = corr_pixel
        print(
            f'Set annotation: {self.data.filename} pixel {self.target_coord} == {corr_filename} pixel {corr_pixel}'
        )

    def find_corr_pixels_onclick(self, event):
        if not event.dblclick:
            return
        try:
            coord = (int(event.ydata), int(event.xdata))
            utils.update_highlight(self.data.highlight, coord)
        except TypeError as e:
            return
        # Update target coordinates and position on mesh
        self.target_coord = coord
        self.target_pos = self.data.pos[coord]

        print('------------------------------------------')
        print(f'Clicked pixel: {coord}, target coordinate: {self.target_pos}')
        if all(self.target_pos == 0):
            self._clear_all_highlights()
        else:
            self.find_corr_pixels(self.target_pos)

    def find_corr_pixels(self, target_pos):
        for filepath, data in self.overlapping_data.items():
            dd, ii = data.pos_kdtree.query(target_pos)
            corr_pixel = np.unravel_index(ii, data.pos_shape)
            corr_pos = data.pos[corr_pixel]
            if dd > self.neighbour_thresh or all(corr_pos == 0):
                utils.update_highlight(
                    self.overlapping_data[filepath].highlight)
                continue
            print(
                f'Corr pixel: {corr_pixel} in {data.filename}\n'
                f'target pos: {target_pos}, corr_pos: {corr_pos} distance = {dd}'
            )
            utils.update_highlight(self.overlapping_data[filepath].highlight,
                                   corr_pixel, self.colors['target_pixel'])

    def _clear_all_highlights(self):
        for plot in self.overlapping_data.values():
            utils.update_highlight(plot.highlight)


class SSSPlotData():
    """Represents an interactive sss_meas_data plot and its corresponding data"""
    def __init__(self,
                 filepath,
                 figsize,
                 normalize_image=True,
                 highlight_color='red'):
        self.filepath = filepath
        self.filename = utils.filepath_to_filename(filepath)
        self.normalize_image = normalize_image
        self.highlight_color = highlight_color

        # Load data
        self.data = sss_meas_data.read_single(filepath)
        self.waterfall_image = self.data.sss_waterfall_image
        self.pos = utils.stack_hits_xyz_to_pos(self.data)
        self.pos_shape = self.pos.shape[:-1]
        self.pos_kdtree = KDTree(self.pos.reshape(-1, 3))

        # Artists
        self.fig, self.ax, self.highlight = self.plot_data(figsize)
        # Callback reference
        self.cid = None

    def plot_data(self, figsize):
        """Plot the waterfall_image and the highlight (position clicked) and
        return the Axes object and Line2D object."""
        fig, ax = plt.subplots(1, figsize=figsize)
        ax.set_title(self.filename, wrap=True)
        image = self.waterfall_image
        if self.normalize_image:
            image = utils.normalize_waterfall_image(image)
        ax.imshow(image)
        highlight, = ax.plot([], [], 'o', color=self.highlight_color, ms=5)
        return fig, ax, highlight

    def register_figure_callback(self, callback, event='button_press_event'):
        """Register callback function"""
        self.cid = self.fig.canvas.mpl_connect(event, callback)
