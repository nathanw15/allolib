# sets:
#   ADDITIONAL_INCLUDE_DIRS
#   ADDITIONAL_LIBRARIES
# 	ADDITIONAL_HEADERS
# 	ADDITIONAL_SOURCES

# al_path needs to be set prior to calling this script

if (AL_WINDOWS)

  FIND_PATH(FREEIMAGE_INCLUDE_PATH FreeImage.h
    ${al_path}/dependencies/FreeImage/Dist/x64
    DOC "The directory where FreeImage.h resides")
  FIND_LIBRARY(FREEIMAGE_LIBRARY
    NAMES FreeImage freeimage
    PATHS ${al_path}/dependencies/FreeImage/Dist/x64
    DOC "The FreeImage library")

  if (FREEIMAGE_INCLUDE_PATH AND FREEIMAGE_LIBRARY)
    SET( FREEIMAGE_FOUND TRUE)
  else ()
    SET( FREEIMAGE_FOUND FALSE)
  endif ()

else ()

  list(APPEND CMAKE_MODULE_PATH
    ${al_path}/cmake_modules/find_scripts
  )

  find_package(FreeImage) # uses cmake_modules/find_scripts/FindFreeImage.cmake
                          # and sets:
                          #     FREEIMAGE_FOUND
                          #     FREEIMAGE_INCLUDE_PATH
                          #     FREEIMAGE_LIBRARY

endif (AL_WINDOWS)

if (FREEIMAGE_FOUND)
  message("found freeimage")
  set(FREEIMAGE_HEADERS
    ${al_path}/include/al/util/al_Image.hpp
  )
  set(FREEIMAGE_SOURCES
    ${al_path}/src/util/al_Image.cpp
  )
endif (FREEIMAGE_FOUND)

set(ADDITIONAL_INCLUDE_DIRS
  ${FREEIMAGE_INCLUDE_PATH}
)

set(ADDITIONAL_LIBRARIES
  ${FREEIMAGE_LIBRARY}
)

set(ADDITIONAL_HEADERS
  ${FREEIMAGE_HEADERS}
)

set(ADDITIONAL_SOURCES
  ${FREEIMAGE_SOURCES}
)