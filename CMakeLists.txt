cmake_minimum_required(VERSION 3.6)

project(allolib)
set(al_path ${CMAKE_CURRENT_SOURCE_DIR})

include(cmake_modules/configure_platform.cmake)
# sets: AL_MACOS || AL_LINUX || AL_WINDOWS, and PLATFORM_DEFINITION
include(cmake_modules/find_core_dependencies.cmake)
# GLEW::GLEW PkgConfig::GLFW ${OPENGL_gl_LIBRARY}
include(cmake_modules/find_additional_dependencies.cmake)
# sets: ADDITIONAL_INCLUDE_DIRS, ADDITIONAL_LIBRARIES,
#       ADDITIONAL_HEADERS, ADDITIONAL_SOURCES
include(cmake_modules/external.cmake)
# sets: EXTERNAL_INCLUDE_DIRS, EXTERNAL_SRC, EXTERNAL_DEFINITIONS
#       EXTERNAL_LIBRARIES
include(cmake_modules/basic_flags.cmake)
# sets: basic_flags
include(cmake_modules/al_files.cmake)
# sets: al_headers, al_sources

set(headers
  ${al_headers} ${ADDITIONAL_HEADERS}
)

set(sources
  ${al_sources} ${EXTERNAL_SRC} ${ADDITIONAL_SOURCES}
)

set(dirs_to_include
  ${ADDITIONAL_INCLUDE_DIRS} ${EXTERNAL_INCLUDE_DIRS}
)

set(libs_to_link
  ${ADDITIONAL_LIBRARIES} ${EXTERNAL_LIBRARIES}
)

set(definitions
  ${PLATFORM_DEFINITION} ${EXTERNAL_DEFINITIONS}
)

set(flags
  ${basic_flags}
)

# multi configuration generators
if (DEFINED CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_CONFIGURATION_TYPES "Debug;Release")
endif ()

add_library(al STATIC ${headers} ${sources})

set_target_properties(al PROPERTIES
    DEBUG_POSTFIX                    _debug
    CXX_STANDARD                     14
    CXX_STANDARD_REQUIRED            ON
    ARCHIVE_OUTPUT_DIRECTORY         ${CMAKE_CURRENT_SOURCE_DIR}/lib
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG   ${CMAKE_CURRENT_SOURCE_DIR}/lib
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/lib
)

target_compile_options(al PUBLIC ${flags})
target_compile_definitions(al PUBLIC ${definitions})
target_include_directories(al PUBLIC ${al_path}/include ${dirs_to_include})
target_link_libraries(al
    ${libs_to_link}
    GLEW::GLEW
    PkgConfig::GLFW
    ${OPENGL_gl_LIBRARY}
)

# --- Testing -----------------------------------------
add_subdirectory(test)