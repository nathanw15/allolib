cmake_minimum_required(VERSION 3.6)
project(allolib)
set(al_path ${CMAKE_CURRENT_SOURCE_DIR})

set(AudioAPI "rtaudio" CACHE STRING "Library for Audio IO")
set_property(CACHE AudioAPI PROPERTY STRINGS rtaudio portaudio dummy)

include(cmake/al_files.cmake)
include(cmake/find_core_dependencies.cmake)
include(cmake/oscpack.cmake)
include(cmake/rtmidi.cmake)

if (AudioAPI STREQUAL "rtaudio")
  include(cmake/rtaudio.cmake)
  set(AUDIO_INCLUDE_DIRS ${RTAUDIO_INCLUDE_DIRS})
  set(AUDIO_SRC ${RTAUDIO_SRC})
  set(AUDIO_LIBRARIES ${RTAUDIO_LIBRARIES})
  set(AUDIO_DEFINITIONS ${RTAUDIO_DEFINITIONS} -DAL_AUDIO_RTAUDIO)

elseif (AudioAPI STREQUAL "portaudio")
  include(cmake/portaudio.cmake)
  set(AUDIO_INCLUDE_DIRS ${PORTAUDIO_INCLUDE_DIRS})
  set(AUDIO_SRC "")
  set(AUDIO_LIBRARIES ${PORTAUDIO_LIBRARIES})
  set(AUDIO_DEFINITIONS -DAL_AUDIO_PORTAUDIO)

elseif (AudioAPI STREQUAL "dummy")
  set(AUDIO_INCLUDE_DIRS "")
  set(AUDIO_SRC "")
  set(AUDIO_LIBRARIES "")
  set(AUDIO_DEFINITIONS -DAL_AUDIO_DUMMY)  
endif()

# multi configuration generators
if (DEFINED CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_CONFIGURATION_TYPES "Debug;Release")
endif ()

add_library(al STATIC
  ${al_headers} ${al_sources} 
  ${OSCPACK_SRC} ${RTMIDI_SRC} ${AUDIO_SRC}
)

set_target_properties(al PROPERTIES
    DEBUG_POSTFIX                    _debug
    CXX_STANDARD                     14
    CXX_STANDARD_REQUIRED            ON
    ARCHIVE_OUTPUT_DIRECTORY         ${CMAKE_CURRENT_SOURCE_DIR}/lib
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG   ${CMAKE_CURRENT_SOURCE_DIR}/lib
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/lib
)

target_compile_options(al PRIVATE
  $<$<PLATFORM_ID:Windows>:>
  $<$<PLATFORM_ID:Darwin>:-Wall>
  $<$<PLATFORM_ID:Linux>:-Wall>
)

target_compile_definitions(al PUBLIC
  "$<$<PLATFORM_ID:Windows>:AL_WINDOWS>"
  "$<$<PLATFORM_ID:Darwin>:AL_OSX;AL_UNIX>"
  "$<$<PLATFORM_ID:Linux>:AL_LINUX;AL_UNIX>"
  ${RTMIDI_DEFINITIONS}
  ${AUDIO_DEFINITIONS}
)

foreach (dir include ${OSCPACK_INCLUDE_DIR} ${RTMIDI_INCLUDE_DIR} ${AUDIO_INCLUDE_DIRS})
    target_include_directories(al PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${dir}>"
        "$<INSTALL_INTERFACE:${dir}>"
    )
endforeach (dir)

target_link_libraries(al PUBLIC
    ${OPENGL_gl_LIBRARY}
    GLEW::GLEW
    PkgConfig::GLFW
    FreeImage::FreeImage
    ${RTMIDI_LIBRARIES}
    ${AUDIO_LIBRARIES}
)

if (${BUILD_TEST})
  add_subdirectory(test)
endif ()

set(CMAKE_INSTALL_PREFIX ${al_path})

install(TARGETS al
  EXPORT al-targets
  RUNTIME DESTINATION al/bin
  LIBRARY DESTINATION al/lib
  ARCHIVE DESTINATION al/lib
)

install(EXPORT al-targets DESTINATION al/cmake)
install(DIRECTORY include DESTINATION al)

install(FILES cmake/find_core_dependencies.cmake DESTINATION al/cmake)
install(FILES cmake/FindFreeImage.cmake DESTINATION al/cmake)
install(FILES cmake/app.cmake DESTINATION al/cmake)

install(DIRECTORY external/oscpack/ip DESTINATION al/external/oscpack)
install(DIRECTORY external/oscpack/osc DESTINATION al/external/oscpack)

if (AudioAPI STREQUAL "rtaudio")
    install(FILES external/rtaudio/RtAudio.h DESTINATION al/external/rtaudio)
endif ()

install(FILES external/rtmidi/rtmidi_c.h external/rtmidi/RtMidi.h
        DESTINATION al/external/rtmidi)

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    install(DIRECTORY dependencies DESTINATION al)
endif ()