# pybathy_maps

# pybathy_maps.draw_map
Data structure for constructing and viewing a bathymetry mesh and for draping the mesh with sidescan data
## bathy_map_image
```python
bathy_map_image(self)
```
Class for constructing mesh from multibeam data
### draw_back_scatter_map
```python
bathy_map_image.draw_back_scatter_map(self)
```
draw_back_scatter_map(self: pybathy_maps.draw_map.bathy_map_image, arg0: List[mbes_ping]) -> None

Draw back scatter map from mbes_ping::PingsT

### draw_height_map
```python
bathy_map_image.draw_height_map(self)
```
draw_height_map(self: pybathy_maps.draw_map.bathy_map_image, arg0: List[mbes_ping]) -> None

Draw height map from mbes_ping::PingsT

### draw_targets
```python
bathy_map_image.draw_targets(self)
```
draw_targets(self: pybathy_maps.draw_map.bathy_map_image, arg0: Dict[unicode, Tuple[float, float]], arg1: cv::Scalar_<double>) -> None

Draw point targets from dict of points

### draw_track
```python
bathy_map_image.draw_track(self)
```
draw_track(self: pybathy_maps.draw_map.bathy_map_image, arg0: List[mbes_ping]) -> None

Draw vehicle track from mbes_ping::PingsT

### save_image
```python
bathy_map_image.save_image(self)
```
save_image(self: pybathy_maps.draw_map.bathy_map_image, arg0: unicode) -> None

Save image to file

# pybathy_maps.mesh_map
Data structure for constructing and viewing a bathymetry mesh and for draping the mesh with sidescan data
## bathy_map_mesh
```python
bathy_map_mesh(self)
```
Class for constructing mesh from multibeam data
### display_height_map
```python
bathy_map_mesh.display_height_map(self)
```
display_height_map(self: pybathy_maps.mesh_map.bathy_map_mesh, arg0: numpy.ndarray[float64[m, n]]) -> None

Display height map using opencv

### display_mesh
```python
bathy_map_mesh.display_mesh(self)
```
display_mesh(self: pybathy_maps.mesh_map.bathy_map_mesh, arg0: numpy.ndarray[float64[m, n]], arg1: numpy.ndarray[int32[m, n]]) -> None

Display mesh using igl viewer

### height_map_from_pings
```python
bathy_map_mesh.height_map_from_pings(self)
```
height_map_from_pings(self: pybathy_maps.mesh_map.bathy_map_mesh, arg0: List[mbes_ping], arg1: float) -> Tuple[numpy.ndarray[float64[m, n]], numpy.ndarray[float64[2, 2]]]

Construct height map from mbes_ping::PingsT

### mesh_from_height_map
```python
bathy_map_mesh.mesh_from_height_map(self)
```
mesh_from_height_map(self: pybathy_maps.mesh_map.bathy_map_mesh, arg0: numpy.ndarray[float64[m, n]], arg1: numpy.ndarray[float64[2, 2]]) -> Tuple[numpy.ndarray[float64[m, n]], numpy.ndarray[int32[m, n]]]

Construct mesh from height map

### mesh_from_pings
```python
bathy_map_mesh.mesh_from_pings(self)
```
mesh_from_pings(self: pybathy_maps.mesh_map.bathy_map_mesh, arg0: List[mbes_ping], arg1: float) -> Tuple[numpy.ndarray[float64[m, n]], numpy.ndarray[int32[m, n]], numpy.ndarray[float64[2, 2]]]

Construct mesh from mbes_ping::PingsT

### overlay_sss
```python
bathy_map_mesh.overlay_sss(self)
```
overlay_sss(self: pybathy_maps.mesh_map.bathy_map_mesh, arg0: numpy.ndarray[float64[m, n]], arg1: numpy.ndarray[int32[m, n]], arg2: numpy.ndarray[float64[2, 2]], arg3: List[xtf_sss_ping]) -> numpy.ndarray[float64[m, n]]

Overlay xtf_sss_ping::PingsT sidescan data on the mesh

