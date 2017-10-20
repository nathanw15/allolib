# user project's CMakeLists.txt file includes this script

# this script needs following to be predefined:
#         app_name
#         app_path
#         app_files_list
#         al_path
#         app_include_dirs (optional)
#         app_link_libs (optional)

set(AudioAPI "rtaudio" CACHE STRING "Library for Audio IO")
set_property(CACHE AudioAPI PROPERTY STRINGS rtaudio portaudio dummy)

include(${al_path}/cmake/find_core_dependencies.cmake)
include(${al_path}/cmake/al-targets.cmake)

# multi configuration generators
if (DEFINED CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_CONFIGURATION_TYPES "Debug;Release")
endif ()

add_executable(${app_name} ${app_files_list})

set_target_properties(${app_name} PROPERTIES
    DEBUG_POSTFIX _debug
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
    RUNTIME_OUTPUT_DIRECTORY ${app_path}/bin
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${app_path}/bin
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${app_path}/bin
)

target_compile_options(al PRIVATE
  $<$<PLATFORM_ID:Windows>:>
  $<$<PLATFORM_ID:Darwin>:-Wall>
  $<$<PLATFORM_ID:Linux>:-Wall>
)

# target_compile_definitions(${app_name} PRIVATE ${definitions})
# target_include_directories(${app_name} PRIVATE ${dirs_to_include})
target_link_libraries(${app_name} al)

#[[
if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  # when run from Visual Studio, working directory is where the solution is by default
  # set it to app output directory
  set_target_properties(${app_name} PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY ${app_path}/bin)
  # startup project is `ALL_BUILD` by default so we change it to app project
  set_directory_properties(PROPERTIES VS_STARTUP_PROJECT ${app_name})

  # post build events for copying dlls
  set(post_build_command
    robocopy ${al_path}/dependencies/glew/bin/Release/x64 ${app_path}/bin glew32.dll &
    robocopy ${al_path}/dependencies/glfw/lib-vc2015 ${app_path}/bin glfw3.dll &
  )

  if (USE_PORTAUDIO)
    list(APPEND post_build_command
      robocopy ${al_path}/dependencies/portaudio/ ${app_path}/bin portaudio_x64.dll &
    )
  endif (USE_PORTAUDIO)

  if (FREEIMAGE_FOUND)
    list(APPEND post_build_command
      robocopy ${al_path}/dependencies/FreeImage/Dist/x64 ${app_path}/bin FreeImage.dll &
    )
  endif (FREEIMAGE_FOUND)
  
  list(APPEND post_build_command
    IF %ERRORLEVEL% LEQ 1 exit 0
  )

  add_custom_command(
    TARGET ${app_name}
    POST_BUILD
    COMMAND ${post_build_command}
  )
endif ()
]]