data_tools
==========

Data structures for parsing and storing different
kinds of maritime data, and to convert them into
common std_data data structures.

The code below shows a simple example of reading
a folder of gsf files, converting them into the
standard multibeam format, and saving the produced
bathymetric map as an image.

.. code:: python

    from auvlib.data_tools import std_data, gsf_data
    from auvlib.bathy_maps import draw_map
    import sys

    gsf_pings = gsf_data.gsf_mbes_ping.parse_folder(sys.argv[1]) # parse folder of gsf data
    mbes_pings = gsf_data.convert_pings(gsf_pings) # convert to std_data pings

    d = draw_map.BathyMapImage(mbes_pings, 500, 500) # create a bathymetry height map
    d.draw_height_map(mbes_pings) # draw the height map
    d.draw_track(mbes_pings) # draw the track of the vehicle
    d.write_image("height_map.png") # save the height map to "height_map.png"

.. toctree::
   :maxdepth: 3

   std_data

   gsf_data

   all_data

   xtf_data

   csv_data

   xyz_data
