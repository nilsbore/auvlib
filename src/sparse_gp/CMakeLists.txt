project(sparse_gp_project)

add_library(rbf_kernel src/rbf_kernel.cpp)

add_library(probit_noise src/probit_noise.cpp)

add_library(gaussian_noise src/gaussian_noise.cpp)

add_library(octave_convenience src/octave_convenience.cpp)

add_library(sparse_gp src/sparse_gp.cpp)

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(rbf_kernel PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(probit_noise PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(gaussian_noise PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(octave_convenience PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(sparse_gp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

message("CMAKE BLA: " ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(sparse_gp rbf_kernel gaussian_noise octave_convenience)

message("INSTALL BGLA: " ${CMAKE_INSTALL_LIBDIR})

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS rbf_kernel probit_noise gaussian_noise octave_convenience sparse_gp EXPORT SparseGPConfig
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT SparseGPConfig DESTINATION share/SparseGP/cmake)

# This makes the project importable from the build directory
export(TARGETS rbf_kernel probit_noise gaussian_noise octave_convenience sparse_gp FILE SparseGPConfig.cmake)
