project(pybathy_maps)

add_library(pymesh_map MODULE src/pymesh_map.cpp)
add_library(pyalign_map MODULE src/pyalign_map.cpp)
add_library(pydraw_map MODULE src/pydraw_map.cpp)
add_library(pypatch_draper MODULE src/pypatch_draper.cpp)
add_library(pymap_draper MODULE src/pymap_draper.cpp)
add_library(pysss_gen_sim MODULE src/pysss_gen_sim.cpp)

target_include_directories(pymesh_map PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(pyalign_map PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(pydraw_map PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(pypatch_draper PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(pymap_draper PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(pysss_gen_sim PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_link_libraries(pymesh_map PRIVATE std_data mesh_map snell_ray_tracing igl::embree ${OpenCV_LIBS} ${BOOST_LIBRARIES} igl::core igl::opengl_glfw -lpthread pybind11::module)
set_target_properties(pymesh_map PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                            OUTPUT_NAME "mesh_map"
                                            SUFFIX "${PYTHON_MODULE_EXTENSION}")

target_link_libraries(pyalign_map PRIVATE std_data xyz_data align_map igl::embree ${OpenCV_LIBS} ${BOOST_LIBRARIES} igl::core igl::opengl_glfw -lpthread pybind11::module)
set_target_properties(pyalign_map PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                            OUTPUT_NAME "align_map"
                                            SUFFIX "${PYTHON_MODULE_EXTENSION}")

target_link_libraries(pydraw_map PRIVATE std_data draw_map ${OpenCV_LIBS} ${BOOST_LIBRARIES} pybind11::module)
set_target_properties(pydraw_map PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                            OUTPUT_NAME "draw_map"
                                            SUFFIX "${PYTHON_MODULE_EXTENSION}")

target_link_libraries(pypatch_draper PRIVATE patch_draper view_draper base_draper igl::embree ${OpenCV_LIBS} ${BOOST_LIBRARIES} igl::core igl::opengl_glfw -lpthread pybind11::module)
set_target_properties(pypatch_draper PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                                  OUTPUT_NAME "patch_draper"
                                                  SUFFIX "${PYTHON_MODULE_EXTENSION}")

target_link_libraries(pymap_draper PRIVATE map_draper view_draper base_draper sss_meas_data igl::embree ${OpenCV_LIBS} ${BOOST_LIBRARIES} igl::core igl::opengl_glfw -lpthread pybind11::module)
set_target_properties(pymap_draper PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                                  OUTPUT_NAME "map_draper"
                                                  SUFFIX "${PYTHON_MODULE_EXTENSION}")

target_link_libraries(pysss_gen_sim PRIVATE sss_gen_sim base_draper igl::embree ${OpenCV_LIBS} ${BOOST_LIBRARIES} igl::core igl::opengl_glfw -lpthread pybind11::module)
set_target_properties(pysss_gen_sim PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                                  OUTPUT_NAME "sss_gen_sim"
                                                  SUFFIX "${PYTHON_MODULE_EXTENSION}")

install(TARGETS pymesh_map pyalign_map pydraw_map pypatch_draper pymap_draper pysss_gen_sim
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}/auvlib/bathy_maps
)  # This is for Windows

install(FILES module/__init__.py DESTINATION ${CMAKE_INSTALL_LIBDIR}/auvlib)
install(FILES module/__init__.py module/data_vis.py module/gen_utils.py DESTINATION ${CMAKE_INSTALL_LIBDIR}/auvlib/bathy_maps)
