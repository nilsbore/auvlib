# pysonar_tracing

Library for tracing sonar beams through horizontal layers with different sound speeds.
For an example usage, see [this file](https://github.com/nilsbore/auvlib/blob/master/src/pysonar_tracing/examples/test_ray_tracing.py).
Most of the functionality is captured by the following snippet:
```python
from pysonar_tracing import snell_ray_tracing
import numpy as np

end_points = np.array([[ 3., -40.1],
                       [13., -40.1],
                       [23., -40.1],
                       [33., -40.1],
                       [43., -32.1],
                       [53., -22.1],
                       [63., -15.1],
                       [73., -22.1],
                       [83., -30.1],
                       [93., -40.1]])
layer_speeds = np.array([0.8, 0.9, 1.1, 1.3])
layer_depths = np.array([-10., -20., -30.])

end_times, layer_widths = snell_ray_tracing.trace_multiple_layers(layer_depths, layer_speeds, end_points)
snell_ray_tracing.visualize_rays(end_points, layer_depths, layer_widths, -45., True)
```

For the full documentation, see below.

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
visualize_rays(arg0: numpy.ndarray[float64[m, n]], arg1: numpy.ndarray[float64[m, 1]], arg2: numpy.ndarray[float64[m, n]], arg3: float, arg4: bool, arg5: bool) -> None

Visualize traces layers using opencv

