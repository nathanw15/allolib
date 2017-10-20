#
# Try to find the FreeImage library and include path.
# Once done this will define
#
# FREEIMAGE_FOUND
# FREEIMAGE_INCLUDE_PATH
# FREEIMAGE_LIBRARY
#

IF (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	# ...
	
ELSE ()

	FIND_PATH( FREEIMAGE_INCLUDE_PATH FreeImage.h
		/usr/include
		/usr/local/include
		/sw/include
		/opt/local/include
		DOC "The directory where FreeImage.h resides")

	FIND_LIBRARY( FREEIMAGE_LIBRARY
		NAMES FreeImage freeimage
		PATHS
		/usr/lib64
		/usr/lib
		/usr/local/lib64
		/usr/local/lib
		/sw/lib
		/opt/local/lib
		DOC "The FreeImage library")

	SET(FREEIMAGE_LIBRARIES ${FREEIMAGE_LIBRARY})

	if (FREEIMAGE_INCLUDE_PATH AND FREEIMAGE_LIBRARY)
		set(FREEIMAGE_FOUND TRUE)
	else ()
		set(FREEIMAGE_FOUND FALSE)
	endif ()

	add_library(FreeImage::FreeImage SHARED IMPORTED)
	set_target_properties(FreeImage::FreeImage PROPERTIES
	    INTERFACE_INCLUDE_DIRECTORIES
	        ${FREEIMAGE_INCLUDE_PATH}
	    IMPORTED_LOCATION
	        ${FREEIMAGE_LIBRARY}
	)

ENDIF ()