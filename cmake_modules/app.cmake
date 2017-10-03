# user project's CMakeLists.txt file includes this script

# this script needs following to be predefined:
#         app_name
#         app_path
#         app_files_list
#         app_include_dirs (can be skipped)
#         app_link_libs (can be skipped)
#         al_path

include(${al_path}/cmake_modules/configure_platform.cmake)
# sets: AL_MACOS || AL_LINUX || AL_WINDOWS, and PLATFORM_DEFINITION
include(${al_path}/cmake_modules/find_core_dependencies.cmake)
# GLEW::GLEW PkgConfig::GLFW ${OPENGL_gl_LIBRARY}
include(${al_path}/cmake_modules/find_additional_dependencies.cmake)
# sets: ADDITIONAL_INCLUDE_DIRS, ADDITIONAL_LIBRARIES,
#       ADDITIONAL_HEADERS, ADDITIONAL_SOURCES
include(${al_path}/cmake_modules/external.cmake)
# sets: EXTERNAL_INCLUDE_DIRS, EXTERNAL_SRC, EXTERNAL_DEFINITIONS
#       EXTERNAL_LIBRARIES
include(${al_path}/cmake_modules/basic_flags.cmake)
# sets: basic_flags

set(dirs_to_include
  ${al_path}/include
  ${app_include_dirs}
  ${ADDITIONAL_INCLUDE_DIRS}
  ${EXTERNAL_INCLUDE_DIRS}
)

set(libs_to_link
  ${app_link_libs}
  ${ADDITIONAL_LIBRARIES}
  ${EXTERNAL_LIBRARIES}
)

set(definitions
  ${PLATFORM_DEFINITION}
  ${EXTERNAL_DEFINITIONS}
)

set(flags
  ${basic_flags}
)

# --- setup app target ------------------------------------

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

target_compile_options(${app_name} PUBLIC ${flags})
target_compile_definitions(${app_name} PRIVATE ${definitions})
target_include_directories(${app_name} PRIVATE ${dirs_to_include})

if (AL_WINDOWS)
  target_link_libraries(${app_name} debug ${al_path}/lib/al_debug.lib optimized ${al_path}/lib/al.lib)
else()
  target_link_libraries(${app_name} debug ${al_path}/lib/libal_debug.a optimized ${al_path}/lib/libal.a)
endif (AL_WINDOWS)
target_link_libraries(${app_name}
    ${libs_to_link}
    GLEW::GLEW
    PkgConfig::GLFW
    ${OPENGL_gl_LIBRARY}
)

if (AL_WINDOWS)
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

  if (USE_APR)
    list(APPEND post_build_command
      robocopy ${al_path}/dependencies/apr/ ${app_path}/bin libapr-1.dll &
    )
  endif (USE_APR)

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
endif (AL_WINDOWS)