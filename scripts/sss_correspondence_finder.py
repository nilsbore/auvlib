import os
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import KDTree
from auvlib.bathy_maps.map_draper import sss_meas_data
import json
from collections import OrderedDict


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
        fig = SSS_Plot(target_filepath,
                       overlapping_filepaths,
                       figsize=figsize,
                       normalize_image=normalize_image)
        plt.show()


class SSS_Plot():
    def __init__(self,
                 filepath,
                 overlapping_filepaths,
                 figsize,
                 neighbour_thresh=.5,
                 normalize_image=True):
        self.neighbour_thresh = neighbour_thresh
        self.filepath = filepath
        self.filepath_to_filename = lambda filepath: os.path.basename(
            os.path.normpath(filepath))
        self.normalize_image = normalize_image

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
        image = data['waterfall_image']
        if self.normalize_image:
            image = self._normalize_waterfall_image(image)
        ax.imshow(image)
        highlight, = ax.plot([], [], 'o', color='red', ms=5)
        return {'ax': ax, 'highlight': highlight}

    def _normalize_waterfall_image(self, waterfall_image, a_max=3):
        waterfall_image = waterfall_image.copy()
        col_mean = waterfall_image.mean(axis=0)
        waterfall_image = np.divide(waterfall_image,
                                    col_mean,
                                    where=[col_mean != 0.])
        clipped_image = np.clip(waterfall_image, a_min=0, a_max=a_max)
        return clipped_image

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
