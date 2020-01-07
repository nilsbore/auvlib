project(data_tools)

add_library(data_transforms src/transforms.cpp)

# Add some libraries
add_library(submaps src/submaps.cpp)

add_library(std_data src/std_data.cpp)

add_library(benchmark src/benchmark.cpp)

add_library(navi_data src/navi_data.cpp)

if(AUVLIB_WITH_GSF)
  add_library(gsf_data src/gsf_data.cpp)
  add_executable(test_submap_tracks src/test_submap_tracks.cpp)
endif()

add_library(csv_data src/csv_data.cpp)

add_library(xtf_data src/xtf_data.cpp)

add_library(jsf_data src/jsf_data.cpp)

add_library(lat_long_utm src/lat_long_utm.cpp)

add_library(all_data src/all_data.cpp)

add_library(xyz_data src/xyz_data.cpp)

add_executable(test_xtf src/test_xtf.cpp)

add_executable(test_all src/test_all.cpp)

add_executable(test_jsf src/test_jsf.cpp)


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

target_include_directories(std_data PUBLIC
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

if(AUVLIB_WITH_GSF)
  target_include_directories(gsf_data PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      PRIVATE src)
endif()

target_include_directories(csv_data PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(xtf_data PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(jsf_data PUBLIC
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

target_include_directories(xyz_data PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

if(MSVC)
  set(EXTRA_BOOST_LIBS "")
else()
  set(EXTRA_BOOST_LIBS ${Boost_LIBRARIES})
endif()

target_link_libraries(test_xtf xtf_data std_data navi_data ${OpenCV_LIBS} ${EXTRA_BOOST_LIBS})

target_link_libraries(test_all all_data ${OpenCV_LIBS} ${EXTRA_BOOST_LIBS})

target_link_libraries(test_jsf jsf_data ${OpenCV_LIBS} ${EXTRA_BOOST_LIBS})

# Link the libraries
target_link_libraries(submaps eigen_cereal ${EXTRA_BOOST_LIBS}) # ${PCL_LIBRARIES})

target_link_libraries(std_data PUBLIC eigen_cereal ${EXTRA_BOOST_LIBS})

target_link_libraries(benchmark PUBLIC eigen_cereal std_data ${OpenCV_LIBS})

target_link_libraries(navi_data PUBLIC eigen_cereal data_transforms) # ${PCL_LIBRARIES})

target_link_libraries(csv_data PUBLIC std_data navi_data)

if(AUVLIB_WITH_GSF)
  target_link_libraries(gsf_data PUBLIC gsf std_data navi_data lat_long_utm csv_data)
  target_link_libraries(test_submap_tracks gsf_data std_data navi_data ${OpenCV_LIBS} ${EXTRA_BOOST_LIBS})
endif()

target_link_libraries(xtf_data PUBLIC std_data navi_data xtf_reader lat_long_utm ${OpenCV_LIBS})

target_link_libraries(jsf_data PUBLIC std_data lat_long_utm xtf_data ${OpenCV_LIBS})

target_link_libraries(all_data navi_data lat_long_utm csv_data ${OpenCV_LIBS})

target_link_libraries(xyz_data std_data ${EXTRA_BOOST_LIBS})

set(AUVLIB_DATA_TOOLS_LIBS data_transforms submaps std_data benchmark navi_data csv_data xtf_data jsf_data all_data xyz_data lat_long_utm)

if(AUVLIB_WITH_GSF)
  set(AUVLIB_DATA_TOOLS_LIBS ${AUVLIB_DATA_TOOLS_LIBS} gsf_data)
endif()

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS ${AUVLIB_DATA_TOOLS_LIBS} EXPORT DataToolsConfig
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT DataToolsConfig DESTINATION share/DataTools/cmake)

# This makes the project importable from the build directory
export(TARGETS ${AUVLIB_DATA_TOOLS_LIBS} FILE DataToolsConfig.cmake)
