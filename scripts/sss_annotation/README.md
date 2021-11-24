# SSS Annotation Tool
The `sss_annotation` package contains tools to annotate corresponding pixels in side-scan sonar
data of `sss_meas_data` structure. The `sss_meas_data` structure can be obtained by draping raw
side-scan sonar files, such as .XTF files, onto a mesh using `auvlib`'s functionalities.

# Set up
1. Install Auvlib according to [instructions](https://github.com/nilsbore/auvlib/)
2. Install `matplotlib` and `scikit-image` packages, e.g. using `pip install matplotlib scikit-image`.

# How to use it
The `SSSFolderAnnotator` class in module `sss_correspondence_finder` is the main entry point for
creating and visualizing annotations. Below is an example snippet.

```python
from sss_annotation.sss_correspondence_finder import SSSFolderAnnotator
import matplotlib.pyplot as plt

draping_res_folder = [path/to/draping/results/folder/with/sss_meas_data/files]
annotator = SSSFolderAnnotator(draping_res_folder)

# Print the existing correspondence annotations
# The correspondence_annotations is a dict with key=unique id and
# value=(a dictionary with key=filename, value=(ping_nbr, col_nbr))
print(annotator.annotations_manager.correspondence_annotations)

# Print the existing keypoints
# The keypoint is a dict with key=filename and value=(a dictionary with
# key=pixel position, value=a list of the corresponding uuid, i.e. multiple
# correspondences entries of the same keypoint are allowed and recorded)."""
print(annotator.annotations_manager.keypoints)

# To start annotating the file corresponding to annotator.data_files[idx]
# This plots the sss_waterfall_image for annotator.data_files[idx] (figure no.1) and the
# potentially overlapping images found in the folder (figure no.>1).
# - Double click on figure 1 to find corresponding pixels based on the XYZ position
#   of the pixel based on draping results (shown as a red dot across all images)
# - To register the correspondence annotation and potentially correct the location
#   of matching pixels in the corresponding images, double click on the desired location
#   in the corresponding images
# - All annotations will be written back to the annotation file once all figures are
#   closed
idx = 0
annotator.plot_sss_waterfall_image_for_annotation(idx)

# Plot keypoints for file annotator.data_files[idx]
annotator.plot_keypoints(idx)

# Plot correspondence annotations for (idx1, idx2)
idx1, idx2 = 0, 1
annotator.plot_correspondences(idx1, idx2)

plt.show()
```
