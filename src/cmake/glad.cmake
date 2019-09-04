add_library(auvlib_glad libigl/external/glad/src/glad.c)
target_include_directories(auvlib_glad PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libigl/external/glad/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE libigl/external/glad/src)
target_link_libraries(auvlib_glad PUBLIC ${CMAKE_DL_LIBS})
# Generate position independent code
set_target_properties(auvlib_glad PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(TARGETS auvlib_glad EXPORT AuvlibGladConfig
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows
install(DIRECTORY libigl/external/glad/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT AuvlibGladConfig DESTINATION share/AuvlibGlad/cmake)

export(TARGETS auvlib_glad FILE AuvlibGladConfig.cmake)
