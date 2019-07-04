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

add_library(align_map src/align_map.cpp)

add_library(drape_mesh src/drape_mesh.cpp)

add_library(base_draper src/base_draper.cpp)

add_library(patch_draper src/patch_draper.cpp)

add_library(map_draper src/map_draper.cpp)

add_library(patch_views src/patch_views.cpp)

add_library(sss_map_image src/sss_map_image.cpp)

add_library(sss_meas_data src/sss_meas_data.cpp)

add_library(sss_gen_sim src/sss_gen_sim.cpp)

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
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(align_map PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(drape_mesh PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(base_draper PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(patch_draper PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(map_draper PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(patch_views PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(sss_map_image PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(sss_meas_data PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(sss_gen_sim PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

#add_dependencies(mesh_map libembree)

# Link the libraries
target_link_libraries(draw_map std_data ${OpenCV_LIBS})

#target_link_libraries(mesh_map std_data igl::embree ${OpenCV_LIBS} glad ${GLFW3_LIBRARY} ${OPENGL_LIBRARY} ${OPENGL_glu_LIBRARY} -lpthread)
target_link_libraries(mesh_map std_data ${OpenCV_LIBS} igl::core igl::opengl_glfw -lpthread -ltinyxml2) # ${TinyXML2_LIBRARIES})

target_link_libraries(align_map mesh_map std_data xyz_data igl::core igl::opengl_glfw -lpthread) # ${TinyXML2_LIBRARIES})

target_link_libraries(test_mesh std_data gsf_data xtf_data csv_data navi_data mesh_map draw_map patch_draper ${OpenCV_LIBS} cxxopts)

target_link_libraries(drape_mesh snell_ray_tracing xtf_data igl::embree igl::core igl::opengl_glfw)

target_link_libraries(patch_views eigen_cereal ${OpenCV_LIBS})

target_link_libraries(sss_map_image eigen_cereal xtf_data ${OpenCV_LIBS})

target_link_libraries(sss_meas_data eigen_cereal xtf_data ${OpenCV_LIBS})

target_link_libraries(base_draper drape_mesh xtf_data patch_views mesh_map ${OpenCV_LIBS} igl::core igl::opengl_glfw -lpthread)

target_link_libraries(patch_draper drape_mesh base_draper xtf_data patch_views ${OpenCV_LIBS} igl::core igl::opengl_glfw -lpthread)

target_link_libraries(map_draper drape_mesh base_draper xtf_data sss_map_image sss_meas_data ${OpenCV_LIBS} igl::core igl::opengl_glfw -lpthread)

target_link_libraries(sss_gen_sim drape_mesh base_draper xtf_data sss_map_image ${OpenCV_LIBS} igl::core igl::opengl_glfw -lpthread ${OpenCV_LIBS})


# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS draw_map mesh_map align_map drape_mesh patch_draper base_draper map_draper patch_views sss_map_image sss_meas_data sss_gen_sim EXPORT BathyMapsConfig
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT BathyMapsConfig DESTINATION share/BathyMaps/cmake)

# This makes the project importable from the build directory
export(TARGETS draw_map mesh_map align_map drape_mesh patch_draper base_draper map_draper patch_views sss_map_image sss_meas_data sss_gen_sim FILE BathyMapsConfig.cmake)
