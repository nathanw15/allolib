set (test_src
    src/main.cpp
)

add_executable(al_tests ${test_src})
set_target_properties(al_tests PROPERTIES
	DEBUG_POSTFIX                    _debug
	CXX_STANDARD                     14
	CXX_STANDARD_REQUIRED            ON
	RUNTIME_OUTPUT_DIRECTORY         ${CMAKE_CURRENT_SOURCE_DIR}/../bin
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   ${CMAKE_CURRENT_SOURCE_DIR}/../bin
	RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/../bin
)
target_include_directories(al_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../external/catch)
target_link_libraries(al_tests al)