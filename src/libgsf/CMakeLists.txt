project(libgsf_project)

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -O3")

# Must use GNUInstallDirs to install libraries into correct
# locations on all platforms.
#include(GNUInstallDirs)

add_definitions(
    -D__USE_LARGEFILE64
    -D_LARGEFILE_SOURCE
    -D_LARGEFILE64_SOURCE
)

add_library(gsf_enc src/gsf_enc.c)

add_library(gsf_dec src/gsf_dec.c)

add_library(gsf_indx src/gsf_indx.c)

#add_library(gsf_info src/gsf_info.cpp)

add_library(gsf src/gsf.c src/gsf_geo.c src/gsf_info.c)

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(gsf_enc PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(gsf_dec PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(gsf_indx PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(gsf PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

#target_include_directories(sparse_gp PUBLIC
#    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#    $<INSTALL_INTERFACE:include>
#    PRIVATE src)

target_link_libraries(gsf gsf_enc gsf_dec gsf_indx)

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS gsf gsf_enc gsf_dec gsf_indx EXPORT LibGSFConfig
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT LibGSFConfig DESTINATION share/LibGSF/cmake)

# This makes the project importable from the build directory
export(TARGETS gsf gsf_enc gsf_dec gsf_indx FILE LibGSFConfig.cmake)
