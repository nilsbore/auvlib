import os
from collections import OrderedDict
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import KDTree
from auvlib.bathy_maps.map_draper import sss_meas_data
import json

from sss_plot import SSSAnnotationPlot
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
        """Plots an SSS waterfall image and the potentially overlapping SSS images."""
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
        annotation_plot = SSSAnnotationPlot(target_filepath,
                                            overlapping_filepaths,
                                            figsize=figsize,
                                            normalize_image=normalize_image)
        plt.show()
        return annotation_plot
