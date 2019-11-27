project(pydata_tools)

message("Python dir: " ${PYTHON_INSTALL_PATH})

#pybind11_add_module(pystd_data src/pystd_data.cpp)
add_library(pystd_data MODULE src/pystd_data.cpp)
if(AUVLIB_WITH_GSF)
  add_library(pygsf_data MODULE src/pygsf_data.cpp)
endif()
add_library(pyxtf_data MODULE src/pyxtf_data.cpp)
add_library(pyjsf_data MODULE src/pyjsf_data.cpp)
add_library(pycsv_data MODULE src/pycsv_data.cpp)
add_library(pyall_data MODULE src/pyall_data.cpp)
add_library(pyxyz_data MODULE src/pyxyz_data.cpp)
add_library(pybenchmark MODULE src/pybenchmark.cpp)

target_include_directories(pystd_data PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

if(AUVLIB_WITH_GSF)
  target_include_directories(pygsf_data PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      PRIVATE src)
endif()

target_include_directories(pyxtf_data PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(pyjsf_data PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(pycsv_data PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(pyall_data PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(pyxyz_data PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(pybenchmark PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)


#target_link_libraries(pystd_data std_data eigen_cereal)
target_link_libraries(pystd_data PRIVATE std_data navi_data eigen_cereal ${BOOST_LIBRARIES} pybind11::module)
set_target_properties(pystd_data PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                            OUTPUT_NAME "std_data"
                                            SUFFIX "${PYTHON_MODULE_EXTENSION}")

if(AUVLIB_WITH_GSF)
  target_link_libraries(pygsf_data PRIVATE std_data gsf_data eigen_cereal ${BOOST_LIBRARIES} pybind11::module)
  set_target_properties(pygsf_data PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                              OUTPUT_NAME "gsf_data"
                                              SUFFIX "${PYTHON_MODULE_EXTENSION}")
endif()

target_link_libraries(pyxtf_data PRIVATE std_data xtf_data eigen_cereal ${BOOST_LIBRARIES} pybind11::module)
set_target_properties(pyxtf_data PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                            OUTPUT_NAME "xtf_data"
                                            SUFFIX "${PYTHON_MODULE_EXTENSION}")

target_link_libraries(pyjsf_data PRIVATE std_data jsf_data eigen_cereal ${BOOST_LIBRARIES} pybind11::module)
set_target_properties(pyjsf_data PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                            OUTPUT_NAME "jsf_data"
                                            SUFFIX "${PYTHON_MODULE_EXTENSION}")

target_link_libraries(pycsv_data PRIVATE std_data csv_data eigen_cereal ${BOOST_LIBRARIES} pybind11::module)
set_target_properties(pycsv_data PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                            OUTPUT_NAME "csv_data"
                                            SUFFIX "${PYTHON_MODULE_EXTENSION}")

target_link_libraries(pyall_data PRIVATE std_data all_data eigen_cereal ${BOOST_LIBRARIES} pybind11::module)
set_target_properties(pyall_data PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                            OUTPUT_NAME "all_data"
                                            SUFFIX "${PYTHON_MODULE_EXTENSION}")

target_link_libraries(pyxyz_data PRIVATE std_data xyz_data eigen_cereal ${BOOST_LIBRARIES} pybind11::module)
set_target_properties(pyxyz_data PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                            OUTPUT_NAME "xyz_data"
                                            SUFFIX "${PYTHON_MODULE_EXTENSION}")

target_link_libraries(pybenchmark PRIVATE std_data benchmark eigen_cereal ${BOOST_LIBRARIES} pybind11::module)
set_target_properties(pybenchmark PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                             OUTPUT_NAME "benchmark"
                                             SUFFIX "${PYTHON_MODULE_EXTENSION}")

set(AUVLIB_PYDATA_TOOLS_LIBS pystd_data pyxtf_data pyjsf_data pycsv_data pyall_data pyxyz_data pybenchmark)
if(AUVLIB_WITH_GSF)
  set(AUVLIB_PYDATA_TOOLS_LIBS ${AUVLIB_PYDATA_TOOLS_LIBS} pygsf_data)
endif()

install(TARGETS ${AUVLIB_PYDATA_TOOLS_LIBS}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}/auvlib/data_tools
)  # This is for Windows

install(FILES module/__init__.py DESTINATION ${CMAKE_INSTALL_LIBDIR}/auvlib)
install(FILES module/__init__.py module/utils.py DESTINATION ${CMAKE_INSTALL_LIBDIR}/auvlib/data_tools)
