project(data_tools)

add_library(data_transforms src/transforms.cpp)

# Add some libraries
add_library(submaps src/submaps.cpp)

add_library(data_structures src/data_structures.cpp)

add_library(navi_data src/navi_data.cpp)

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(data_transforms PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(submaps PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(data_structures PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(navi_data PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

# Link the libraries
target_link_libraries(submaps ${PCL_LIBRARIES})

target_link_libraries(data_structures PUBLIC sparse_gp rbf_kernel gaussian_noise)

target_link_libraries(navi_data PUBLIC sparse_gp data_transforms ${PCL_LIBRARIES})

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS data_transforms submaps data_structures navi_data EXPORT DataToolsConfig
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT DataToolsConfig DESTINATION share/DataTools/cmake)

# This makes the project importable from the build directory
export(TARGETS data_transforms submaps data_structures navi_data FILE DataToolsConfig.cmake)
