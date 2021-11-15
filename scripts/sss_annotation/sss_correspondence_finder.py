import os
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import KDTree
from auvlib.bathy_maps.map_draper import sss_meas_data
import json
from collections import OrderedDict

from sss_plot import SSSPlotData
from utils import update_highlight


class SSSDrapingFolderParser():
    """Parse a side-scan sonar draping results folder containing .cereal files
    of sss_meas_data objects."""
    def __init__(self, draping_res_folder):
        self.folder = draping_res_folder
        self.data_files = self._get_data_files_dict()
        self.num_data_files = len(self.data_files)
        self.bounds_filename = 'bounds.json'
        self.bounds_dict = self._get_bounds_dict()

    def _get_data_files_dict(self):
        """Return a dict of .cereal file paths found under self.folder.
        The dictionary has key = file name, value = file path."""
        files = OrderedDict()
        for filename in sorted(os.listdir(self.folder)):
            if not os.path.splitext(filename)[1] == '.cereal':
                continue
            files[filename] = os.path.join(self.folder, filename)
        print(f'Found {len(files)} .cereal data files.\nFiles: {files}')
        return files

    def _get_bounds_dict(self):
        """Load or construct a dictionary containing the xyz bounds for
        each of the .cereal data file"""
        bounds_file = os.path.join(self.folder, self.bounds_filename)

        if os.path.exists(bounds_file):
            with open(bounds_file, 'r') as f:
                return json.load(f)
        return self._construct_and_store_bounds_dict(bounds_file)

    def _construct_and_store_bounds_dict(self, bounds_file):
        """Construct a dictionary containing the xyz bounds for each of
        the .cereal data file under self.folder and store the dictionary
        to file"""
        bounds_dict = {}
        for filename, filepath in self.data_files.items():
            meas_data = sss_meas_data.read_single(filepath)
            bounds_dict[filename] = {
                'x':
                self._get_bounds_for_array(meas_data.sss_waterfall_hits_X),
                'y':
                self._get_bounds_for_array(meas_data.sss_waterfall_hits_Y),
                'z':
                self._get_bounds_for_array(meas_data.sss_waterfall_hits_Z),
            }
        with open(bounds_file, 'w') as f:
            json.dump(bounds_dict, f)
        return bounds_dict

    def _get_bounds_for_array(self, array):
        """Returns (min(max(0, min(array))), max(array))"""
        values = array[array > 0]
        if len(values) <= 0:
            return (array.min().astype(np.float64),
                    array.max().astype(np.float64))
        return (values.min().astype(np.float64),
                values.max().astype(np.float64))

    def _range_overlap(self, range1, range2):
        """Checks whether two ranges given in (min, max) tuples overlap"""
        return range2[0] <= range1[0] <= range2[1] or range1[0] <= range2[
            0] <= range1[1]

    def get_overlapping_filenames(self, target_filename):
        """Return a list of file names that are potentially overlapping with
        the given target_filename"""
        idx_bounds = self.bounds_dict[target_filename]
        overlapping_filenames = []
        for filename, bounds in self.bounds_dict.items():
            if filename == target_filename:
                continue
            if any([
                    self._range_overlap(bounds[axis], idx_bounds[axis])
                    for axis in bounds.keys()
            ]):
                overlapping_filenames.append(filename)
        print(f'\nChosen file: {target_filename}\n'
              f'Number of overlapping files: {len(overlapping_filenames)}\n'
              f'Overlapping files: {overlapping_filenames}')
        return overlapping_filenames

    def plot_sss_waterfall_image(self,
                                 idx,
                                 figsize=(5, 10),
                                 normalize_image=True):
        """Plots an SSS waterfall image and the potentially overlapping SSS images.
        Uses a SSS_Plot object to handle interactions."""
        if not 0 <= idx < self.num_data_files:
            raise ValueError(
                f'Valid index range: (0, {self.num_data_files - 1})')

        target_filename = list(self.data_files.keys())[idx]
        target_filepath = self.data_files[target_filename]
        # overlapping_filepaths = [target_filename]
        overlapping_filenames = self.get_overlapping_filenames(target_filename)
        overlapping_filepaths = [
            self.data_files[name] for name in overlapping_filenames
        ]
        fig = SSSAnnotationPlot(target_filepath,
                                overlapping_filepaths,
                                figsize=figsize,
                                normalize_image=normalize_image)
        plt.show()


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

        #TODO: register callback!
        self.data = SSSPlotData(filepath, figsize, normalize_image)
        self.data.register_figure_callback(self.find_corr_pixels_onclick)

        self.num_overlapping_data = len(overlapping_filepaths)
        self.overlapping_data = self.plot_overlapping_data(
            overlapping_filepaths, figsize, normalize_image)

        self.target_pos = None
        self.target_coord = None
        self.colors = {'target_pixel': 'red', 'annotation': 'yellow'}

        self.annotations = {}

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
        try:
            coord = (int(event.ydata), int(event.xdata))
        except TypeError as e:
            return
        print('------------------------------------------')
        clicked_axis = event.inaxes
        for k, v in self.overlapping_data.items():
            if v.ax == clicked_axis:
                name = v.filename
                print(f'Clicked on plot for {name}, pixel = {coord}')
                update_highlight(v.highlight, coord, self.colors['annotation'])

    #TODO: separate click from drag and zoom events
    def find_corr_pixels_onclick(self, event):
        try:
            coord = (int(event.ydata), int(event.xdata))
            update_highlight(self.data.highlight, coord)
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
                update_highlight(self.overlapping_data[filepath].highlight)
                continue
            print(
                f'Corr pixel: {corr_pixel} in {data.filename}\n'
                f'target pos: {target_pos}, corr_pos: {corr_pos} distance = {dd}'
            )
            update_highlight(self.overlapping_data[filepath].highlight,
                             corr_pixel)

    def _clear_all_highlights(self):
        for plot in self.overlapping_data.values():
            update_highlight(plot.highlight)
