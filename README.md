# auvlib

Tools for reading AUV deployment data files and for
processing and visualization of side scan and multibeam data.

## Dependencies

On Ubuntu 16.04, just use the following command to install all dependencies:
```
sudo apt-get install libcereal-dev libglfw3-dev libceres-dev
```

## Building

Once cloned, get the libigl submodule via `git submodule init`, `git submodule update`.
Then go into the `libigl` folder and execute `git submodule update --init external/embree`
and `git submodule update --init external/glfw`.
Finally, create a `build` folder in the repo root, and run
```
cmake -DCMAKE_INSTALL_PREFIX=../install ..
```
within the `build` folder. Then run `make` and `make install`.
You should now have a compiled version of auvlib in the folder
`/path/to/auvlib/install`. When done, please execute
```
export PYTHONPATH=$PYTHONPATH:/path/to/auvlib/install/lib
```
in any terminal where you want to use the python version of
the library, or add this line to your `~/.bashrc`.

## Using as a python library

Python is the preferred interface for auvlib. In general, the python bindings have more
complete documentation and supports most of the use cases of the c++ library.

### Simple python example

As an example, in the snippet below, we read multibeam data from a `.gsf` file,
and create an image with the vehicle track and a multibeam height map.

```python
from pydata_tools import std_data, gsf_data
from pybathy_maps import draw_map
import sys

gsf_pings = gsf_data.gsf_mbes_ping.parse_folder(sys.argv[1]) # parse folder of gsf data
mbes_pings = gsf_data.convert_pings(gsf_pings) # convert to std_data pings

d = draw_map.BathyMapImage(mbes_pings, 500, 500) # create a bathymetry height map
d.draw_height_map(mbes_pings) # draw the height map
d.draw_track(mbes_pings) # draw the track of the vehicle
d.write_image("height_map.png") # save the height map to "height_map.png"
```

### Advanced functionality

```python
from pydata_tools import std_data, gsf_data, xtf_data, csv_data, utils
from pybathy_maps import mesh_map, patch_draper, data_vis
import sys, os, math
import numpy as np

sensor_yaw = 5.*math.pi/180. # rotation of sidescan wrt nav frame
sensor_offset = np.array([2., -1.5, 0.]) # translation of sidescan wrt nav frame

gsf_pings = utils.parse_or_load_gsf(sys.argv[1]) # parse_or_load* functions will just parse the first time
mbes_pings = gsf_data.convert_pings(gsf_pings) # convert to std_data pings
V, F, bounds = mesh_map.mesh_from_pings(mbes_pings, 0.5) # generate a bathymetry mesh

xtf_pings = utils.parse_or_load_xtf(sys.argv[2]) # load sidescan pings
nav_entries = utils.parse_or_load_csv(sys.argv[3]) # load gps nav entries
xtf_pings = csv_data.convert_matched_entries(xtf_pings, nav_entries) # match sidescan with gps
xtf_pings = xtf_data.correct_sensor_offset(xtf_pings, sensor_offset) # correct for sidescan translation
sound_speeds = csv_data.csv_asvp_sound_speed.parse_file(sys.argv[4]) # parse sound speed file

viewer = patch_draper.PatchDraper(V, F, xtf_pings, bounds, sound_speeds) # create a draper object
viewer.set_sidescan_yaw(sensor_yaw) # set the rotation of sensor wrt nav frame
viewer.set_vehicle_mesh(*patch_draper.get_vehicle_mesh()) # add a vehicle model for visualization
viewer.set_patch_callback(lambda patch: data_vis.plot_patch_views([patch])) # add a plotter callback
viewer.show() # show the visualization and drape
```

### Python documentation and resources

The project directories contains examples and documentation, see
[pybathy_maps](https://github.com/nilsbore/auvlib/tree/master/src/pybathy_maps),
[pydata_tools](https://github.com/nilsbore/auvlib/tree/master/src/pydata_tools) or
[pysonar_tracing](https://github.com/nilsbore/auvlib/tree/master/src/pysonar_tracing).

## Using as a c++ library

First, initialize the submodules, same as for the previous section. For using auvlib as a library in an external project,
[check out the example projects](https://github.com/nilsbore/auvlib/tree/master/example_projects).
If you just want to use auvlib for reading data, please see the minimal [data project](https://github.com/nilsbore/auvlib/tree/master/example_projects/data_project).

For more complete documentation on C++ library usage, see [the overview document](https://github.com/nilsbore/auvlib/blob/master/docs/cpp_usage.md).
