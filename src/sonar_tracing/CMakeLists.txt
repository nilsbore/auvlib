project(sonar_tracing)

#include(ExternalProject)

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

include_directories(${LIBIGL_INCLUDE_DIR})
include_directories(${EMBREE_INCLUDE_DIRS})

if(NOT MSVC AND AUVLIB_WITH_CERES)
  find_package(Ceres REQUIRED)
  include_directories(${CERES_INCLUDE_DIRS})
  add_definitions(-DWITH_CERES)
endif()

#include_directories(../../libigl/external/glad/include)
#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../libigl/external/embree/include)
#link_directories(${CMAKE_BINARY_DIR}/embree)
add_executable(test_ray_tracing src/test_ray_tracing.cpp)

# Add some libraries
add_library(snell_ray_tracing src/snell_ray_tracing.cpp)

add_library(bathy_tracer src/bathy_tracer.cpp)

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(snell_ray_tracing PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(bathy_tracer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

#add_dependencies(mesh_map libembree)

# Link the libraries
target_link_libraries(snell_ray_tracing std_data ${CERES_LIBRARIES} ${OpenCV_LIBS} -lpthread)

target_link_libraries(test_ray_tracing snell_ray_tracing ${CERES_LIBRARIES} ${OpenCV_LIBS} -lpthread cxxopts)

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS snell_ray_tracing bathy_tracer EXPORT SonarTracingConfig
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT SonarTracingConfig DESTINATION share/SonarTracing/cmake)

# This makes the project importable from the build directory
export(TARGETS snell_ray_tracing bathy_tracer FILE SonarTracingConfig.cmake)
