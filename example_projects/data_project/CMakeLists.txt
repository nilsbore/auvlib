cmake_minimum_required(VERSION 2.8)
project(data_project)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -O3")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Eigen3 REQUIRED)
#find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

find_package(LibXTF REQUIRED)
find_package(LibGSF REQUIRED)
find_package(EigenCereal REQUIRED)
find_package(DataTools REQUIRED)

include_directories(include ${EIGEN_INCLUDE_DIRS})

add_executable(example_reader src/example_reader.cpp)
target_link_libraries(example_reader gsf_data xtf_data csv_data navi_data all_data)
