project(pybathy_maps)

add_library(pymesh_map MODULE src/pymesh_map.cpp)
add_library(pydraw_map MODULE src/pydraw_map.cpp)

target_include_directories(pymesh_map PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(pydraw_map PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_link_libraries(pymesh_map PRIVATE data_structures mesh_map snell_ray_tracing igl::embree ${OpenCV_LIBS} ${BOOST_LIBRARIES} igl::core igl::opengl_glfw -lpthread pybind11::module)
set_target_properties(pymesh_map PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                            OUTPUT_NAME "mesh_map"
                                            SUFFIX "${PYTHON_MODULE_EXTENSION}")

target_link_libraries(pydraw_map PRIVATE data_structures draw_map ${OpenCV_LIBS} ${BOOST_LIBRARIES} pybind11::module)
set_target_properties(pydraw_map PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                            OUTPUT_NAME "draw_map"
                                            SUFFIX "${PYTHON_MODULE_EXTENSION}")

install(TARGETS pymesh_map pydraw_map
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}/pybathy_maps
)  # This is for Windows

install(FILES module/__init__.py DESTINATION ${CMAKE_INSTALL_LIBDIR}/pybathy_maps)