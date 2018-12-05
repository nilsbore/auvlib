project(pybathy_maps)

add_library(pymesh_map MODULE src/pymesh_map.cpp)
add_library(pydraw_map MODULE src/pydraw_map.cpp)
add_library(pydraping_viewer MODULE src/pydraping_viewer.cpp)
add_library(pydraping_image MODULE src/pydraping_image.cpp)

target_include_directories(pymesh_map PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(pydraw_map PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(pydraping_viewer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(pydraping_image PUBLIC
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

target_link_libraries(pydraping_viewer PRIVATE draping_viewer draping_generator igl::embree ${OpenCV_LIBS} ${BOOST_LIBRARIES} igl::core igl::opengl_glfw -lpthread pybind11::module)
set_target_properties(pydraping_viewer PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                                  OUTPUT_NAME "draping_viewer"
                                                  SUFFIX "${PYTHON_MODULE_EXTENSION}")

target_link_libraries(pydraping_image PRIVATE draping_image draping_generator igl::embree ${OpenCV_LIBS} ${BOOST_LIBRARIES} igl::core igl::opengl_glfw -lpthread pybind11::module)
set_target_properties(pydraping_image PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                                  OUTPUT_NAME "draping_image"
                                                  SUFFIX "${PYTHON_MODULE_EXTENSION}")

install(TARGETS pymesh_map pydraw_map pydraping_viewer pydraping_image
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}/pybathy_maps
)  # This is for Windows

install(FILES module/__init__.py DESTINATION ${CMAKE_INSTALL_LIBDIR}/pybathy_maps)
