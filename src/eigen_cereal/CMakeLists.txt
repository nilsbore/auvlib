project(libeigen_cereal)

add_library(eigen_cereal INTERFACE)
#target_sources(eigen_cereal INTERFACE eigen_cereal/eigen_cereal.h)

#include_directories(include)
#target_include_directories(eigen_cereal PUBLIC
    #    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    #    $<INSTALL_INTERFACE:include>)

target_include_directories(eigen_cereal INTERFACE
    #"${PROJECT_SOURCE_DIR}/include"
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

install(TARGETS eigen_cereal EXPORT EigenCerealConfig
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT EigenCerealConfig DESTINATION share/EigenCereal/cmake)

# This makes the project importable from the build directory
export(TARGETS eigen_cereal FILE EigenCerealConfig.cmake)
