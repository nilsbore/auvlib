# pybathy_maps

Library for creating images and meshes from a dataset with multibeam data and vehicle poses.
There are examples of how to use it with
[images](https://github.com/nilsbore/auvlib/blob/master/src/pybathy_maps/examples/draw_height_map.py) and
[meshes](https://github.com/nilsbore/auvlib/blob/master/src/pybathy_maps/examples/view_gsf_mesh.py).

For the full documentation, see below.

# pybathy_maps.draw_map
Data structure for constructing and viewing a bathymetry mesh and for draping the mesh with sidescan data
## BathyMapImage
```python
BathyMapImage(self)
```
Class for constructing mesh from multibeam data
### draw_back_scatter_map
```python
BathyMapImage.draw_back_scatter_map(self)
```
draw_back_scatter_map(self: pybathy_maps.draw_map.BathyMapImage, arg0: List[std_data::mbes_ping]) -> None

Draw back scatter map from mbes_ping::PingsT

### draw_height_map
```python
BathyMapImage.draw_height_map(self)
```
draw_height_map(self: pybathy_maps.draw_map.BathyMapImage, arg0: List[std_data::mbes_ping]) -> None

Draw height map from mbes_ping::PingsT

### draw_targets
```python
BathyMapImage.draw_targets(self)
```
draw_targets(self: pybathy_maps.draw_map.BathyMapImage, arg0: Dict[unicode, Tuple[float, float]], arg1: cv::Scalar_<double>) -> None

Draw point targets from dict of points

### draw_track
```python
BathyMapImage.draw_track(self)
```
draw_track(self: pybathy_maps.draw_map.BathyMapImage, arg0: List[std_data::mbes_ping]) -> None

Draw vehicle track from mbes_ping::PingsT

### show
```python
BathyMapImage.show(self)
```
show(self: pybathy_maps.draw_map.BathyMapImage) -> None

Show the drawn bathy map

### write_image
```python
BathyMapImage.write_image(self)
```
write_image(self: pybathy_maps.draw_map.BathyMapImage, arg0: unicode) -> None

Save image to file

# pybathy_maps.mesh_map
Data structure for constructing and viewing a bathymetry mesh and for draping the mesh with sidescan data
## height_map_from_pings
```python
height_map_from_pings(self)
```
height_map_from_pings(arg0: List[std_data::mbes_ping], arg1: float) -> Tuple[numpy.ndarray[float64[m, n]], numpy.ndarray[float64[2, 2]]]

Construct height map from mbes_ping::PingsT

## mesh_from_height_map
```python
mesh_from_height_map(self)
```
mesh_from_height_map(arg0: numpy.ndarray[float64[m, n]], arg1: numpy.ndarray[float64[2, 2]]) -> Tuple[numpy.ndarray[float64[m, n]], numpy.ndarray[int32[m, n]]]

Construct mesh from height map

## mesh_from_pings
```python
mesh_from_pings(self)
```
mesh_from_pings(arg0: List[std_data::mbes_ping], arg1: float) -> Tuple[numpy.ndarray[float64[m, n]], numpy.ndarray[int32[m, n]], numpy.ndarray[float64[2, 2]]]

Construct mesh from mbes_ping::PingsT

## show_height_map
```python
show_height_map(self)
```
show_height_map(arg0: numpy.ndarray[float64[m, n]]) -> None

Display height map using opencv

## show_mesh
```python
show_mesh(self)
```
show_mesh(arg0: numpy.ndarray[float64[m, n]], arg1: numpy.ndarray[int32[m, n]]) -> None

Display mesh using igl viewer

# pybathy_maps.patch_draper
Functions for draping a mesh with sidescan data
## BaseDraper
```python
BaseDraper(self)
```
Base class for draping sidescan pings onto a bathymetry mesh
### set_ray_tracing_enabled
```python
BaseDraper.set_ray_tracing_enabled(self)
```
set_ray_tracing_enabled(self: pybathy_maps.patch_draper.BaseDraper, arg0: bool) -> None

Set if ray tracing through water layers should be enabled. Takes more time but is recommended if there are large speed differences

### set_sidescan_yaw
```python
BaseDraper.set_sidescan_yaw(self)
```
set_sidescan_yaw(self: pybathy_maps.patch_draper.BaseDraper, arg0: float) -> None

Set yaw correction of sidescan with respect to nav frame

### set_vehicle_mesh
```python
BaseDraper.set_vehicle_mesh(self)
```
set_vehicle_mesh(self: pybathy_maps.patch_draper.BaseDraper, arg0: numpy.ndarray[float64[m, n]], arg1: numpy.ndarray[int32[m, n]], arg2: numpy.ndarray[float64[m, n]]) -> None

Provide the viewer with a vehicle model, purely for visualization

### show
```python
BaseDraper.show(self)
```
show(self: pybathy_maps.patch_draper.BaseDraper) -> None

Start the draping, and show the visualizer

## color_jet_from_mesh
```python
color_jet_from_mesh(self)
```
color_jet_from_mesh(arg0: numpy.ndarray[float64[m, n]]) -> numpy.ndarray[float64[m, n]]

Get a jet color scheme from a vertex matrix

## drape_patches
```python
drape_patches(self)
```
drape_patches(arg0: numpy.ndarray[float64[m, n]], arg1: numpy.ndarray[int32[m, n]], arg2: numpy.ndarray[float64[2, 2]], arg3: List[xtf_data::xtf_sss_ping], arg4: List[csv_data::csv_asvp_sound_speed], arg5: float, arg6: Callable[[pybathy_maps.patch_draper.sss_patch_views], None]) -> List[pybathy_maps.patch_draper.sss_patch_views]

Overlay xtf_sss_ping::PingsT sidescan data on a mesh and get sss_patch_views::ViewsT

## drape_viewer
```python
drape_viewer(self)
```
drape_viewer(arg0: numpy.ndarray[float64[m, n]], arg1: numpy.ndarray[int32[m, n]], arg2: numpy.ndarray[float64[2, 2]], arg3: List[xtf_data::xtf_sss_ping], arg4: List[csv_data::csv_asvp_sound_speed], arg5: float) -> None

Overlay xtf_sss_ping::PingsT sidescan data on a mesh and get sss_patch_views::ViewsT

## get_vehicle_mesh
```python
get_vehicle_mesh(self)
```
get_vehicle_mesh() -> Tuple[numpy.ndarray[float64[m, n]], numpy.ndarray[int32[m, n]], numpy.ndarray[float64[m, n]]]

Get vertices, faces, and colors for vehicle

## PatchDraper
```python
PatchDraper(self)
```
Base class for draping sidescan pings onto a particular point of a bathymetry mesh
### get_patch_views
```python
PatchDraper.get_patch_views(self)
```
get_patch_views(self: pybathy_maps.patch_draper.PatchDraper) -> List[pybathy_maps.patch_draper.sss_patch_views]

Get all the sss_patch_views::PatchesT that have been gathered so far

### set_patch_callback
```python
PatchDraper.set_patch_callback(self)
```
set_patch_callback(self: pybathy_maps.patch_draper.PatchDraper, arg0: Callable[[pybathy_maps.patch_draper.sss_patch_views], None]) -> None

Set the function to be called when all views of a patch have been assembled

### set_ray_tracing_enabled
```python
PatchDraper.set_ray_tracing_enabled(self)
```
set_ray_tracing_enabled(self: pybathy_maps.patch_draper.PatchDraper, arg0: bool) -> None

Set if ray tracing through water layers should be enabled. Takes more time but is recommended if there are large speed differences

### set_sidescan_yaw
```python
PatchDraper.set_sidescan_yaw(self)
```
set_sidescan_yaw(self: pybathy_maps.patch_draper.PatchDraper, arg0: float) -> None

Set yaw correction of sidescan with respect to nav frame

### set_vehicle_mesh
```python
PatchDraper.set_vehicle_mesh(self)
```
set_vehicle_mesh(self: pybathy_maps.patch_draper.PatchDraper, arg0: numpy.ndarray[float64[m, n]], arg1: numpy.ndarray[int32[m, n]], arg2: numpy.ndarray[float64[m, n]]) -> None

Provide the viewer with a vehicle model, purely for visualization

### show
```python
PatchDraper.show(self)
```
show(self: pybathy_maps.patch_draper.PatchDraper) -> None

Start the draping, and show the visualizer

## sss_patch_views
```python
sss_patch_views(self)
```
Class for sidescan views of a patch from different survey lines
### patch_height
Member
### patch_origin
Member
### patch_size
Member
### patch_view_dirs
Member
### patch_view_pos
Member
### read_data
```python
sss_patch_views.read_data(self)
```
read_data(arg0: unicode) -> List[pybathy_maps.patch_draper.sss_patch_views]

Read sss_patch_views::ViewsT from .cereal file

### sss_views
Member
## write_data
```python
write_data(self)
```
write_data(arg0: List[pybathy_maps.patch_draper.sss_patch_views], arg1: unicode) -> None

Write sss_patch_views::ViewsT to .cereal file

# pybathy_maps.map_draper
Functions for draping a mesh with sidescan data
## color_jet_from_mesh
```python
color_jet_from_mesh(self)
```
color_jet_from_mesh(arg0: numpy.ndarray[float64[m, n]]) -> numpy.ndarray[float64[m, n]]

Get a jet color scheme from a vertex matrix

## convert_maps_to_patches
```python
convert_maps_to_patches(self)
```
convert_maps_to_patches(arg0: List[pybathy_maps.map_draper.sss_map_image], arg1: numpy.ndarray[float64[m, n]], arg2: float) -> List[pybathy_maps.patch_draper.sss_patch_views]

Convert sss_map_image::ImagesT to sss_patch_views::ViewsT

## drape_maps
```python
drape_maps(self)
```
drape_maps(arg0: numpy.ndarray[float64[m, n]], arg1: numpy.ndarray[int32[m, n]], arg2: numpy.ndarray[float64[2, 2]], arg3: List[xtf_data::xtf_sss_ping], arg4: List[csv_data::csv_asvp_sound_speed], arg5: float, arg6: float, arg7: Callable[[pybathy_maps.map_draper.sss_map_image], None]) -> List[pybathy_maps.map_draper.sss_map_image]

Overlay xtf_sss_ping::PingsT sidescan data on a mesh and get sss_map_image::ViewsT

## get_vehicle_mesh
```python
get_vehicle_mesh(self)
```
get_vehicle_mesh() -> Tuple[numpy.ndarray[float64[m, n]], numpy.ndarray[int32[m, n]], numpy.ndarray[float64[m, n]]]

Get vertices, faces, and colors for vehicle

## MapDraper
```python
MapDraper(self)
```
Class for draping the whole data set of sidescan pings onto a bathymetry mesh
### get_images
```python
MapDraper.get_images(self)
```
get_images(self: pybathy_maps.map_draper.MapDraper) -> List[pybathy_maps.map_draper.sss_map_image]

Get all the sss_map_image::ImagesT that have been gathered so far

### set_image_callback
```python
MapDraper.set_image_callback(self)
```
set_image_callback(self: pybathy_maps.map_draper.MapDraper, arg0: Callable[[pybathy_maps.map_draper.sss_map_image], None]) -> None

Set the function to be called when an entire sidescan map is done

### set_ray_tracing_enabled
```python
MapDraper.set_ray_tracing_enabled(self)
```
set_ray_tracing_enabled(self: pybathy_maps.map_draper.MapDraper, arg0: bool) -> None

Set if ray tracing through water layers should be enabled. Takes more time but is recommended if there are large speed differences

### set_resolution
```python
MapDraper.set_resolution(self)
```
set_resolution(self: pybathy_maps.map_draper.MapDraper, arg0: float) -> None

Set the resolution of the gathered maps, default is ~3.75

### set_sidescan_yaw
```python
MapDraper.set_sidescan_yaw(self)
```
set_sidescan_yaw(self: pybathy_maps.map_draper.MapDraper, arg0: float) -> None

Set yaw correction of sidescan with respect to nav frame

### set_vehicle_mesh
```python
MapDraper.set_vehicle_mesh(self)
```
set_vehicle_mesh(self: pybathy_maps.map_draper.MapDraper, arg0: numpy.ndarray[float64[m, n]], arg1: numpy.ndarray[int32[m, n]], arg2: numpy.ndarray[float64[m, n]]) -> None

Provide the viewer with a vehicle model, purely for visualization

### show
```python
MapDraper.show(self)
```
show(self: pybathy_maps.map_draper.MapDraper) -> None

Start the draping, and show the visualizer

## sss_map_image
```python
sss_map_image(self)
```
Class for sidescan views of a patch from different survey lines
### bounds
Member
### pos
Member
### read_data
```python
sss_map_image.read_data(self)
```
read_data(arg0: unicode) -> List[pybathy_maps.map_draper.sss_map_image]

Read sss_map_image::ImagesT from .cereal file

### sss_map_image
Member
### sss_waterfall_cross_track
Member
### sss_waterfall_depth
Member
### sss_waterfall_image
Member
## write_data
```python
write_data(self)
```
write_data(arg0: List[pybathy_maps.map_draper.sss_map_image], arg1: unicode) -> None

Write sss_map_image::ImagesT to .cereal file

