cmake_minimum_required (VERSION 2.8.1)

SET(LSM9DS1_VERSION_MAJOR 0)
SET(LSM9DS1_VERSION_MINOR 1)
SET(LSM9DS1_VERSION_PATCH 0)
SET(LSM9DS1_VERSION_STRING ${LSM9DS1_VERSION_MAJOR}.${LSM9DS1_VERSION_MINOR}.${LSM9DS1_VERSION_PATCH})

add_definitions(-DBUILD_VERSION="${LSM9DS1_VERSION_STRING}")

option(BUILD_DOCUMENTATION "Build lsm9ds1 documentation" OFF)

if(BUILD_DOCUMENTATION)

	# add a target to generate API documentation with Doxygen
	find_package(Doxygen)

    if(NOT DOXYGEN_FOUND)
        message(FATAL_ERROR "Doxygen is needed to build the documentation.")
    endif()

    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
    set(doxyfile ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    configure_file(${doxyfile_in} ${doxyfile} @ONLY)

    add_custom_target(doc
        COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM)
endif(BUILD_DOCUMENTATION)

######## Project settings ########
PROJECT(liblsm9ds1)
SET(LICENSE "MIT")


set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DDEBUG=1 -g")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Wall -pedantic -DBUILD_VERSION=${LSM9DS1_VERSION_STRING}")

######## Build and include settings ########
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(
	inc
)

link_directories(
	${LINK_DIRECTORIES}
)

file(GLOB SOURCES
	"src/lsm9ds1.c"
)

file(GLOB PYTHON_EXTENSION_SOURCES
	"src/lsm9ds1_wrap.c"
	"src/lsm9ds1.c"
)

file(GLOB TEST_SOURCES
	"tests/test.c"
)

#Generate the shared library from the sources
add_library(lsm9ds1 SHARED ${SOURCES})

SET_TARGET_PROPERTIES(lsm9ds1 PROPERTIES VERSION ${LSM9DS1_VERSION_STRING}
                                          SOVERSION ${LSM9DS1_VERSION_MAJOR})

add_library(pylsm9ds1 SHARED ${PYTHON_EXTENSION_SOURCES})

enable_testing()
add_executable(lsm9ds1_test ${TEST_SOURCES})
target_link_libraries(lsm9ds1_test cunit)
target_link_libraries(lsm9ds1_test lsm9ds1)

######## Install targets ########
INSTALL(TARGETS lsm9ds1
	DESTINATION /usr/lib/
)

SET_TARGET_PROPERTIES(pylsm9ds1 PROPERTIES PREFIX "")
INSTALL(TARGETS pylsm9ds1
	DESTINATION /usr/lib/python3.5/site-packages
)

INSTALL(TARGETS lsm9ds1_test
	DESTINATION /usr/bin
)
