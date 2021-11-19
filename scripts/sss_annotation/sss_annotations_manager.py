import json
import uuid
import os
from collections import OrderedDict, defaultdict
from sss_annotation.utils import filepath_to_filename


class SSSAnnotationManager:
    """Load and write annotations"""
    def __init__(self, filepath):
        self.correspondence_annotations_file = filepath
        self.correspondence_annotations_filename = filepath_to_filename(
            self.correspondence_annotations_file)

        self.correspondence_annotations = self.load_annotations()
        self.keypoints = self._parse_keypoints_from_correspondence_annotations(
        )

    def load_annotations(self):
        """Load annotation file if exist, construct an OrderedDict otherwise.
        The annotation dictionary has key=id and value=(a dictionary with
        key=filename, value=(ping_nbr, col_nbr))"""
        data = OrderedDict()
        if os.path.exists(self.correspondence_annotations_file):
            with open(self.correspondence_annotations_file, 'r') as f:
                data = json.load(f)
        return data

    def _parse_keypoints_from_correspondence_annotations(self):
        """Parse the correspondence_annotations dictionary and return a
        keypoint dictionary with key=filename and value=(a dictionary with
        key=pixel position, value=a list of the corresponding uuid, i.e. multiple
        correspondences entries of the same keypoint are allowed and recorded)."""
        keypoints = defaultdict(dict)
        for kp_id, values in self.correspondence_annotations.items():
            for filename, pixel in values.items():
                keypoints[filename][tuple(pixel)] = [kp_id]
        return keypoints

    def update_annotation(self, target_filename, annotations):
        """Update the correspondence_annotations and keypoint annotations"""
        for target_pixel, correspondences in annotations.items():
            kp_id = str(uuid.uuid1())
            # Update keypoints dict
            if target_pixel in self.keypoints[target_filename]:
                self.keypoints[target_filename][target_pixel].append(kp_id)
            else:
                self.keypoints[target_filename][target_pixel] = [kp_id]
            # Update correspondence_annotations dict
            corr_dict = {target_filename: target_pixel}

            for corr_filename, corr_pixel in correspondences.items():
                # Update keypoints dict
                if corr_pixel in self.keypoints[corr_filename]:
                    self.keypoints[corr_filename][corr_pixel].append(kp_id)
                else:
                    self.keypoints[corr_filename][corr_pixel] = [kp_id]
                # Update correspondence_annotations dict
                corr_dict[corr_filename] = corr_pixel
            self.correspondence_annotations[kp_id] = corr_dict

    def write_annotations(self):
        with open(self.correspondence_annotations_file, 'w') as f:
            json.dump(self.correspondence_annotations, f)
