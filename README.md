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

### Python example

As an example, in the snippet below, we read multibeam data from a `.gsf` file,
and create an image with the vehicle track and a multibeam height map.

```python
from pydata_tools import std_data, gsf_data
from pybathy_maps import draw_map
import sys

gsf_pings = gsf_data.gsf_mbes_ping.parse_folder(sys.argv[1])
mbes_pings = gsf_data.convert_pings(gsf_pings)

d = draw_map.BathyMapImage(mbes_pings, 500, 500)
d.draw_height_map(mbes_pings)
d.draw_track(mbes_pings)
d.write_image("height_map.png")
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

