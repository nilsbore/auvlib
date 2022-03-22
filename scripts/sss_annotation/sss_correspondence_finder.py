import os
import json
from collections import OrderedDict
import matplotlib.pyplot as plt
import numpy as np
from auvlib.bathy_maps.map_draper import sss_meas_data
from skimage.feature import plot_matches

from sss_annotation.sss_annotations_manager import SSSAnnotationManager
from sss_annotation.sss_plot import SSSAnnotationPlot, SSSPlotData
from sss_annotation.utils import normalize_waterfall_image


class SSSFolderAnnotator():
    """Parse a side-scan sonar draping results folder containing .cereal files
    of sss_meas_data objects."""
    def __init__(self, draping_res_folder):
        self.folder = draping_res_folder
        self.data_files = self._get_data_files_dict()
        self.num_data_files = len(self.data_files)
        self.bounds_filename = 'bounds.json'
        self.bounds_dict = self._get_bounds_dict()

        # Correspondence annotations
        self.correspondence_annotations_filename = 'correspondence_annotations.json'
        self.annotations_manager = SSSAnnotationManager(
            os.path.join(self.folder,
                         self.correspondence_annotations_filename))

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

    def plot_sss_waterfall_image_for_annotation(self,
                                                idx,
                                                figsize=(5, 10),
                                                normalize_image=True):
        """Plots an SSS waterfall image and the potentially overlapping SSS images
        for manual annotations. Write the updated correspondence_annotations to
        the correspondence_annotations_file and return the correspondence_annotations."""
        if not 0 <= idx < self.num_data_files:
            raise ValueError(
                f'Valid index range: (0, {self.num_data_files - 1})')

        target_filename = self._idx_to_filename(idx)
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

        self.annotations_manager.update_annotation(target_filename,
                                                   annotation_plot.annotations)
        self.annotations_manager.write_annotations()
        return self.annotations_manager.correspondence_annotations

    def _idx_to_filename(self, idx):
        return list(self.data_files.keys())[idx]

    def plot_keypoints_for_all_files(self):
        """Plot annotated keypoints for all files in self.folder"""
        for i in range(len(self.data_files)):
            self.plot_keypoints(i)
        plt.show()

    def plot_keypoints(self, idx, figsize=(5, 10), normalize_image=True):
        """Plot annotated keypoints for file number idx. Call plt.show() to
        bring up the image."""
        filename = self._idx_to_filename(idx)
        plot = SSSPlotData(self.data_files[filename], figsize, normalize_image)
        kps = self.annotations_manager.keypoints[filename]
        ping_nbrs, col_nbrs = [], []
        for (ping_nbr, col_nbr) in kps.keys():
            ping_nbrs.append(ping_nbr)
            col_nbrs.append(col_nbr)
        plot.ax.scatter(col_nbrs, ping_nbrs, c='yellow', s=5)

    def plot_correspondences(self, idx1, idx2):
        """Plot annotated correspondences for file number (idx1, idx2). Call
        plt.show() to bring up the image."""
        filename1 = self._idx_to_filename(idx1)
        filename2 = self._idx_to_filename(idx2)
        matched_kps1 = []
        matched_kps2 = []
        no_match_kp1, no_match_kp2 = [], []
        for kp_id, kp_dict in self.annotations_manager.correspondence_annotations.items(
        ):
            if filename1 in kp_dict and filename2 in kp_dict:
                matched_kps1.append(list(kp_dict[filename1]))
                matched_kps2.append(list(kp_dict[filename2]))
            elif filename1 in kp_dict:
                no_match_kp1.append(list(kp_dict[filename1]))
            elif filename2 in kp_dict:
                no_match_kp2.append(list(kp_dict[filename2]))

        matches = np.array([[i, i] for i in range(len(matched_kps1))])
        matched_kps1.extend(no_match_kp1)
        kps1 = np.array(matched_kps1)
        matched_kps2.extend(no_match_kp2)
        kps2 = np.array(matched_kps2)

        fig, ax = plt.subplots(1, 1)
        img1 = normalize_waterfall_image(
            sss_meas_data.read_single(
                self.data_files[filename1]).sss_waterfall_image)
        img2 = normalize_waterfall_image(
            sss_meas_data.read_single(
                self.data_files[filename2]).sss_waterfall_image)
        plot_matches(ax,
                     img1,
                     img2,
                     kps1,
                     kps2,
                     matches,
                     keypoints_color='y',
                     matches_color='g')
        ax.set_title(f'Keypoint annotations for {filename1} and {filename2}')
