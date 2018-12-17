project(data_tools)

add_library(data_transforms src/transforms.cpp)

# Add some libraries
add_library(submaps src/submaps.cpp)

add_library(data_structures src/data_structures.cpp)

add_library(benchmark src/benchmark.cpp)

add_library(navi_data src/navi_data.cpp)

add_library(gsf_data src/gsf_data.cpp)

add_library(csv_data src/csv_data.cpp)

add_library(xtf_data src/xtf_data.cpp)

add_library(lat_long_utm src/lat_long_utm.cpp)

add_library(all_data src/all_data.cpp)

add_executable(test_gsf src/test_gsf.cpp)

add_executable(test_xtf src/test_xtf.cpp)

add_executable(test_submap_tracks src/test_submap_tracks.cpp)

add_executable(test_all src/test_all.cpp)

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

target_include_directories(benchmark PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(navi_data PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(gsf_data PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(csv_data PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(xtf_data PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(lat_long_utm PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(all_data PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_link_libraries(test_gsf gsf)

target_link_libraries(test_xtf xtf_data data_structures navi_data ${OpenCV_LIBS})

target_link_libraries(test_submap_tracks gsf_data data_structures navi_data ${OpenCV_LIBS})

target_link_libraries(test_all all_data ${OpenCV_LIBS})

# Link the libraries
target_link_libraries(submaps eigen_cereal ${PCL_LIBRARIES})

target_link_libraries(data_structures PUBLIC eigen_cereal) # sparse_gp rbf_kernel gaussian_noise ${OpenCV_LIBS})

target_link_libraries(benchmark PUBLIC eigen_cereal data_structures ${OpenCV_LIBS}) #sparse_gp rbf_kernel gaussian_noise ${OpenCV_LIBS})

target_link_libraries(navi_data PUBLIC eigen_cereal data_transforms ${PCL_LIBRARIES}) #sparse_gp data_transforms ${PCL_LIBRARIES})

target_link_libraries(gsf_data PUBLIC gsf data_structures navi_data lat_long_utm)

target_link_libraries(csv_data PUBLIC data_structures navi_data gsf_data xtf_data)

target_link_libraries(xtf_data PUBLIC data_structures navi_data xtf_reader lat_long_utm ${OpenCV_LIBS})

target_link_libraries(all_data navi_data lat_long_utm ${OpenCV_LIBS})

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS data_transforms submaps data_structures benchmark navi_data gsf_data csv_data xtf_data all_data lat_long_utm EXPORT DataToolsConfig
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT DataToolsConfig DESTINATION share/DataTools/cmake)

# This makes the project importable from the build directory
export(TARGETS data_transforms submaps data_structures benchmark navi_data gsf_data csv_data xtf_data all_data lat_long_utm FILE DataToolsConfig.cmake)
