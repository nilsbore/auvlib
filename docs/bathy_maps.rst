bathy_maps
==========

Methods for fusing and registering bathymetric data.

The example below shows how to read multibeam and
sidescan data, and drape the sidescan pings on top
of the bathymetric map.

.. code:: python

    from auvlib.data_tools import std_data, gsf_data, xtf_data, csv_data, utils
    from auvlib.bathy_maps import mesh_map, patch_draper, data_vis
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

.. toctree::
   :maxdepth: 3

   draw_map

   mesh_map

   base_draper

   patch_draper

   map_draper

   sss_gen_sim
