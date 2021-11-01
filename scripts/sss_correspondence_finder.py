import os
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import KDTree
from auvlib.bathy_maps.map_draper import sss_meas_data
import json
from collections import OrderedDict


class SSS_correspondence():
    def __init__(self, draping_res_folder):
        self.folder = draping_res_folder
        self.data_files = self._get_data_files_list()
        self.num_data_files = len(self.data_files)
        self.bounds_filename = 'bounds.json'
        self.bounds_dict = self._get_bounds_dict()

    def _get_data_files_list(self):
        """Return a list of .cereal file paths found under self.folder"""
        files = []
        for filename in os.listdir(self.folder):
            if not os.path.splitext(filename)[1] == '.cereal':
                continue
            files.append(os.path.join(self.folder, filename))
        files = sorted(files)
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
        for filepath in self.data_files:
            meas_data = sss_meas_data.read_single(filepath)
            bounds_dict[filepath] = {
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

    def get_overlapping_filepaths(self, target_filename):
        idx_bounds = self.bounds_dict[target_filename]
        overlapping_filepaths = []
        for filename, bounds in self.bounds_dict.items():
            if filename == target_filename:
                continue
            if any([
                    self._range_overlap(bounds[axis], idx_bounds[axis])
                    for axis in bounds.keys()
            ]):
                overlapping_filepaths.append(filename)
        print(f'\nChosen file: {target_filename}\n'
              f'Number of overlapping files: {len(overlapping_filepaths)}\n'
              f'Overlapping files: {overlapping_filepaths}')
        return overlapping_filepaths

    def plot_sss_waterfall_image(self, idx, figsize=(5, 10)):
        if not 0 <= idx < self.num_data_files:
            raise ValueError(
                f'Valid index range: (0, {self.num_data_files - 1})')

        target_filename = self.data_files[idx]
        # overlapping_filepaths = [target_filename]
        overlapping_filepaths = self.get_overlapping_filepaths(target_filename)
        fig = SSS_Plot(target_filename, overlapping_filepaths, figsize=figsize)
        plt.show()


class SSS_Plot():
    def __init__(self,
                 filepath,
                 overlapping_filepaths,
                 figsize,
                 neighbour_thresh=.5):
        self.neighbour_thresh = neighbour_thresh
        self.filepath = filepath
        self.filepath_to_filename = lambda filepath: os.path.basename(
            os.path.normpath(filepath))

        self.data = self._load_data(self.filepath)
        self.target_pos = None
        self.plot = self.plot_data_and_register_callback(figsize)

        self.overlapping_filepaths = overlapping_filepaths
        self.num_overlapping_data = len(self.overlapping_filepaths)
        self.overlapping_data = self._load_overlapping_data_into_dict()
        self.overlapping_plots = self.plot_overlapping_data(figsize)

    def _load_data(self, filepath):
        data = sss_meas_data.read_single(filepath)
        pos = self._stack_hits_xyz_to_pos(data)
        pos_shape = pos.shape[:-1]
        pos_kdtree = KDTree(pos.reshape(-1, 3))
        return {
            'filename': self.filepath_to_filename(filepath),
            'waterfall_image': data.sss_waterfall_image,
            'pos': pos,
            'pos_shape': pos_shape,
            'pos_kdtree': pos_kdtree
        }

    def _load_overlapping_data_into_dict(self):
        data_dict = OrderedDict()
        for filepath in self.overlapping_filepaths:
            data_dict[filepath] = self._load_data(filepath)
        return data_dict

    def _stack_hits_xyz_to_pos(self, data):
        return np.stack([
            data.sss_waterfall_hits_X, data.sss_waterfall_hits_Y,
            data.sss_waterfall_hits_Z
        ],
                        axis=-1)

    def plot_data_and_register_callback(self, figsize):
        fig, ax = plt.subplots(1, figsize=(figsize))
        plot = self._plot_data(self.data, ax)
        cid = fig.canvas.mpl_connect('button_press_event', self.onclick)
        return plot

    def _plot_data(self, data, ax):
        ax.set_title(data['filename'], wrap=True)
        ax.imshow(data['waterfall_image'])
        highlight, = ax.plot([], [], 'o', color='red', ms=5)
        return {'ax': ax, 'highlight': highlight}

    def plot_overlapping_data(self, figsize):
        overlapping_plots = OrderedDict()
        for i, (filepath, data) in enumerate(self.overlapping_data.items()):
            fig, ax = plt.subplots(1, figsize=figsize)
            overlapping_plots[filepath] = self._plot_data(data, ax)
        return overlapping_plots

    #TODO: separate click from drag and zoom events
    def onclick(self, event):
        try:
            coord = (int(event.ydata), int(event.xdata))
            self._update_highlight(self.plot['highlight'], coord)
        except TypeError as e:
            return
        self.target_pos = self.data['pos'][coord]
        print('------------------------------------------')
        print(f'Clicked pixel: {coord}, target coordinate: {self.target_pos}')
        if all(self.target_pos == 0):
            self._clear_all_highlights()
        else:
            self.find_corr_pixels(self.target_pos)

    def find_corr_pixels(self, target_pos):
        for filepath, data in self.overlapping_data.items():
            dd, ii = data['pos_kdtree'].query(target_pos)
            if dd > self.neighbour_thresh:
                self._update_highlight(
                    self.overlapping_plots[filepath]['highlight'])
                continue
            corr_pixel = np.unravel_index(ii, data['pos_shape'])
            corr_pos = data['pos'][corr_pixel]
            print(
                f'Corr pixel: {corr_pixel} in {self.filepath_to_filename(filepath)}\n'
                f'target pos: {target_pos}, corr_pos: {corr_pos} distance = {dd}'
            )
            self._update_highlight(
                self.overlapping_plots[filepath]['highlight'], corr_pixel)

    def _clear_all_highlights(self):
        for plot in self.overlapping_plots.values():
            self._update_highlight(plot['highlight'])

    def _update_highlight(self, highlight, pixel=([], [])):
        highlight.set_data(pixel[1], pixel[0])
        highlight.figure.canvas.draw_idle()


def test(idx=0):
    folder = '~/Documents/local-corr/data/210209/pp/ETPro/ssh/9-0169to0182/draping-res-no-offset/'
    obj = SSS_correspondence(folder)
    obj.plot_sss_waterfall_image(idx)
