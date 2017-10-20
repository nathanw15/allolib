# sets:
#   CORE_INCLUDE_DIRS
#   CORE_LIBRARIES
#   CORE_LIBRARY_DIRS

# al_path needs to be set prior to calling this script
# FindFreeImage.cmake needs to be in same direcotry with this script
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})

find_package(OpenGL REQUIRED)
# for ${OPENGL_gl_LIBRARY}, include dirs are handled with glew

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")

    add_library(GLEW::GLEW SHARED IMPORTED)
    set_target_properties(GLEW::GLEW PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES
            ${al_path}/dependencies/glew/include
        IMPORTED_IMPLIB
            ${al_path}/dependencies/glew/lib/Release/x64/glew32.lib
        IMPORTED_LOCATION
            ${al_path}/dependencies/glew/bin/Release/x64/glew32.dll
    )

    add_library(PkgConfig::GLFW SHARED IMPORTED)
    set_target_properties(PkgConfig::GLFW PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES
            ${al_path}/dependencies/glfw/include
        IMPORTED_IMPLIB
            ${al_path}/dependencies/glfw/lib-vc2015/glfw3dll.lib
        IMPORTED_LOCATION
            ${al_path}/dependencies/glfw/lib-vc2015/glfw3.dll
    )

    add_library(FreeImage::FreeImage SHARED IMPORTED)
    set_target_properties(FreeImage::FreeImage PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES
            ${al_path}/dependencies/FreeImage/Dist/x64
        IMPORTED_IMPLIB
            ${al_path}/dependencies/FreeImage/Dist/x64/FreeImage.lib
        IMPORTED_LOCATION
            ${al_path}/dependencies/FreeImage/Dist/x64/FreeImage.dll
    )

else () # UNIX

    find_package(GLEW REQUIRED)                            # GLEW::GLEW
    find_package(PkgConfig REQUIRED)
    pkg_search_module(GLFW REQUIRED IMPORTED_TARGET glfw3) # PkgConfig::GLFW
    find_package(FreeImage REQUIRED)                       # FreeImage::FreeImage

endif ()