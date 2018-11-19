# pysonar_tracing

Library for tracing sonar beams through horizontal layers with different sound speeds.
For an example usage, see [this file](https://github.com/nilsbore/auvlib/blob/master/src/pysonar_tracing/examples/test_ray_tracing.py).

# pysonar_tracing.snell_ray_tracing
Data structure for constructing and viewing a bathymetry mesh and for draping the mesh with sidescan data
## trace_multiple_layers
```python
trace_multiple_layers(self)
```
trace_multiple_layers(arg0: numpy.ndarray[float64[m, 1]], arg1: numpy.ndarray[float64[m, 1]], arg2: numpy.ndarray[float64[m, n]]) -> Tuple[numpy.ndarray[float64[m, 1]], numpy.ndarray[float64[m, n]]]

Trace multiple rays to a sequence of layers

## trace_single_layers
```python
trace_single_layers(self)
```
trace_single_layers(arg0: numpy.ndarray[float64[m, 1]], arg1: numpy.ndarray[float64[m, 1]], arg2: numpy.ndarray[float64[2, 1]]) -> Tuple[float, numpy.ndarray[float64[m, 1]]]

Trace single ray through a sequence of layers

## visualize_rays
```python
visualize_rays(self)
```
visualize_rays(arg0: numpy.ndarray[float64[m, n]], arg1: numpy.ndarray[float64[m, 1]], arg2: numpy.ndarray[float64[m, n]], arg3: float, arg4: bool) -> None

Visualize traces layers using opencv

