project(pydata_tools)

message("Python dir: " ${PYTHON_INSTALL_PATH})

#pybind11_add_module(pydata_structures src/pydata_structures.cpp)
add_library(pydata_structures MODULE src/pydata_structures.cpp)
add_library(pygsf_data MODULE src/pygsf_data.cpp)
add_library(pyxtf_data MODULE src/pyxtf_data.cpp)
add_library(pycsv_data MODULE src/pycsv_data.cpp)
add_library(pyall_data MODULE src/pyall_data.cpp)

target_include_directories(pydata_structures PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(pygsf_data PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(pyxtf_data PUBLIC
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


#target_link_libraries(pydata_structures data_structures eigen_cereal)
target_link_libraries(pydata_structures PRIVATE data_structures eigen_cereal ${BOOST_LIBRARIES} pybind11::module)
set_target_properties(pydata_structures PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                                   OUTPUT_NAME "data_structures"
                                                   SUFFIX "${PYTHON_MODULE_EXTENSION}")

target_link_libraries(pygsf_data PRIVATE data_structures gsf_data eigen_cereal ${BOOST_LIBRARIES} pybind11::module)
set_target_properties(pygsf_data PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                            OUTPUT_NAME "gsf_data"
                                            SUFFIX "${PYTHON_MODULE_EXTENSION}")

target_link_libraries(pyxtf_data PRIVATE data_structures xtf_data eigen_cereal ${BOOST_LIBRARIES} pybind11::module)
set_target_properties(pyxtf_data PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                            OUTPUT_NAME "xtf_data"
                                            SUFFIX "${PYTHON_MODULE_EXTENSION}")

target_link_libraries(pycsv_data PRIVATE data_structures csv_data eigen_cereal ${BOOST_LIBRARIES} pybind11::module)
set_target_properties(pycsv_data PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                            OUTPUT_NAME "csv_data"
                                            SUFFIX "${PYTHON_MODULE_EXTENSION}")

target_link_libraries(pyall_data PRIVATE data_structures all_data eigen_cereal ${BOOST_LIBRARIES} pybind11::module)
set_target_properties(pyall_data PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                            OUTPUT_NAME "all_data"
                                            SUFFIX "${PYTHON_MODULE_EXTENSION}")