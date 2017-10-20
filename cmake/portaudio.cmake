# sets:
#   PORTAUDIO_INCLUDE_DIRS
#   PORTAUDIO_LIBRARIES

# al_path needs to be set prior to calling this script

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set(PORTAUDIO_INCLUDE_DIRS ${al_path}/dependencies/portaudio/include)
    set(PORTAUDIO_LIBRARIES ${al_path}/dependencies/portaudio/portaudio_x64.lib)

else () # UNIX
    find_package(PkgConfig REQUIRED)
    pkg_search_module(PORTAUDIO REQUIRED portaudio-2.0)
    
endif ()