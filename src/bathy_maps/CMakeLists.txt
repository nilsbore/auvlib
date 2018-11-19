project(bathy_maps)

include(ExternalProject)

#set(EMBREE_ISPC_SUPPORT OFF)
#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../libigl/external/embree $${CMAKE_CURRENT_BINARY_DIR})

#ExternalProject_Add(libembree
#  PREFIX embree
#  DOWNLOAD_COMMAND ""
#  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../libigl/external/embree
#  BINARY_DIR ${CMAKE_BINARY_DIR}/embree
#  INSTALL_COMMAND ""
#  CMAKE_ARGS -DEMBREE_ISPC_SUPPORT=OFF
#)


#include_directories(../../libigl/external/glad/include)
#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../libigl/external/embree/include)
#link_directories(${CMAKE_BINARY_DIR}/embree)

# Add some libraries
add_library(draw_map src/draw_map.cpp)

add_library(mesh_map src/mesh_map.cpp)

add_executable(test_mesh src/test_mesh.cpp)

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(draw_map PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(mesh_map PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    #$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../libigl/include>
    #$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../libigl/external/glad/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

#add_dependencies(mesh_map libembree)

# Link the libraries
target_link_libraries(draw_map data_structures ${OpenCV_LIBS})

#target_link_libraries(mesh_map data_structures igl::embree ${OpenCV_LIBS} glad ${GLFW3_LIBRARY} ${OPENGL_LIBRARY} ${OPENGL_glu_LIBRARY} -lpthread)
target_link_libraries(mesh_map data_structures snell_ray_tracing igl::embree ${OpenCV_LIBS} igl::core igl::opengl_glfw -lpthread)

target_link_libraries(test_mesh data_structures gsf_data xtf_data csv_data navi_data mesh_map draw_map ${OpenCV_LIBS})

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS draw_map mesh_map EXPORT BathyMapsConfig
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT BathyMapsConfig DESTINATION share/BathyMaps/cmake)

# This makes the project importable from the build directory
export(TARGETS draw_map mesh_map FILE BathyMapsConfig.cmake)
