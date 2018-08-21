project(libxtf_project)

include_directories(include)
#add_definitions(
#    -D__USE_LARGEFILE64
#    -D_LARGEFILE_SOURCE
#    -D_LARGEFILE64_SOURCE
#)

add_library(xtf_reader src/xtf_reader.c)

add_executable(demo_xtf src/demo_xtf.c)
add_executable(demo_xtf_reader src/demo_xtf_reader.cpp)

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(xtf_reader PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

#target_include_directories(sparse_gp PUBLIC
#    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#    $<INSTALL_INTERFACE:include>
#    PRIVATE src)

target_link_libraries(demo_xtf_reader xtf_reader)

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS xtf_reader EXPORT LibXTFConfig
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT LibXTFConfig DESTINATION share/LibXTF/cmake)

# This makes the project importable from the build directory
export(TARGETS xtf_reader FILE LibXTFConfig.cmake)
