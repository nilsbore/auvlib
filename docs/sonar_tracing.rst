sonar_tracing
=============

Methods for ray tracing through water layers with
different sound speeds. See example below.

.. code:: python

    from auvlib.sonar_tracing import snell_ray_tracing
    import numpy as np

    # end points of rays in sensor's coordinate system
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

    # sound speeds in every layer
    layer_speeds = np.array([0.8, 0.9, 1.1, 1.3])
    # depth of layers
    layer_depths = np.array([-10., -20., -30.])

    end_times, layer_widths = snell_ray_tracing.trace_multiple_layers(layer_depths, layer_speeds, end_points)

    snell_ray_tracing.visualize_rays(end_points, layer_depths, layer_widths, -45., True, False)

.. toctree::
   :maxdepth: 3

   snell_ray_tracing
