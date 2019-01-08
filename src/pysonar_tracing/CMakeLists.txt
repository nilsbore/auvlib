project(pysonar_tracing)

add_library(pysnell_ray_tracing MODULE src/pysnell_ray_tracing.cpp)

target_include_directories(pysnell_ray_tracing PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_link_libraries(pysnell_ray_tracing PRIVATE snell_ray_tracing ${OpenCV_LIBS} pybind11::module)
set_target_properties(pysnell_ray_tracing PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                                     OUTPUT_NAME "snell_ray_tracing"
                                                     SUFFIX "${PYTHON_MODULE_EXTENSION}")

install(TARGETS pysnell_ray_tracing
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}/auvlib/sonar_tracing
)  # This is for Windows

install(FILES module/__init__.py DESTINATION ${CMAKE_INSTALL_LIBDIR}/auvlib)
install(FILES module/__init__.py DESTINATION ${CMAKE_INSTALL_LIBDIR}/auvlib/sonar_tracing)
