project(Preprocess)

SET(GCC_PP_COMPILE_FLAGS "-g")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_PP_COMPILE_FLAGS}")

find_package(OpenCV REQUIRED core highgui imgproc imgcodecs)

add_library(sidescan src/sidescan.cpp)

add_library(Timestamp src/Timestamp.cc)

add_executable(tests src/tests.cpp)

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.

target_include_directories(Timestamp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(sidescan PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

if(MSVC)
  set(EXTRA_BOOST_LIBS "")
else()
  set(EXTRA_BOOST_LIBS ${Boost_LIBRARIES})
endif()

target_link_libraries(tests sidescan Timestamp xtf_data std_data navi_data ${OpenCV_LIBS} ${EXTRA_BOOST_LIBS})


# Link the libraries

target_link_libraries(Timestamp PUBLIC)
target_link_libraries(sidescan PUBLIC Timestamp xtf_data std_data navi_data xtf_reader lat_long_utm ${OpenCV_LIBS})

set(AUVLIB_PREPROCESS_LIBS sidescan Timestamp)

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS ${AUVLIB_PREPROCESSS_LIBS} EXPORT PreprocessConfig
	    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
	    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
	    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
#install(EXPORT PreprocessConfig DESTINATION share/Preprocess/cmake)

# This makes the project importable from the build directory
#export(TARGETS ${AUVLIB_PREPROCSS_LIBS} FILE PreprocessConfig.cmake)
