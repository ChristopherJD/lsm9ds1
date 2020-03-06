############################## Project Version ################################

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.1)

SET(LSM9DS1_VERSION_MAJOR 0)
SET(LSM9DS1_VERSION_MINOR 1)
SET(LSM9DS1_VERSION_PATCH 0)
SET(LSM9DS1_VERSION_STRING 
    ${LSM9DS1_VERSION_MAJOR}.${LSM9DS1_VERSION_MINOR}.${LSM9DS1_VERSION_PATCH})
SET(LIB_BUILD_VERSION ${LSM9DS1_VERSION_STRING} CACHE STRING "Build Version")
OPTION(BUILD_VERSION "Set the version being built." ${LIB_BUILD_VERSION})
ADD_DEFINITIONS(-DBUILD_VERSION="${BUILD_VERSION}")
###############################################################################


############################ Doxygen Documentation #############################
OPTION(BUILD_DOCUMENTATION "Build lsm9ds1 doxygen documentation" OFF)

if(BUILD_DOCUMENTATION)

	# add a target to generate API documentation with Doxygen
	FIND_PACKAGE(Doxygen)

    if(NOT DOXYGEN_FOUND)
        MESSAGE(FATAL_ERROR "Doxygen is needed to build the documentation.")
    endif()

    SET(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
    SET(doxyfile ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    CONFIGURE_FILE(${doxyfile_in} ${doxyfile} @ONLY)

    ADD_CUSTOM_TARGET(doc
        COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM)
endif(BUILD_DOCUMENTATION)
###############################################################################

############################ Project Settings #################################
IF(CMAKE_BUILD_TYPE MATCHES Debug)
    message("Debug mode")
    SET(PROJECT liblsm9ds1-dbg)
ELSE()
    message("Release mode")
    SET(PROJECT liblsm9ds1)
ENDIF(CMAKE_BUILD_TYPE MATCHES Debug)

PROJECT(liblsm9ds1)
SET(LICENSE "GPLV3")
###############################################################################

########################### Build and include settings ########################
STRING(CONCAT CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DDEBUG=1 -g")
STRING(CONCAT CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Wall -pedantic")

INCLUDE_DIRECTORIES(
    inc
)

LINK_DIRECTORIES(
    ${LINK_DIRECTORIES}
)

########################## C Library ##########################################
FILE(GLOB HEADERS
    "inc/*.h"
)
FILE(GLOB SOURCES 
    "src/*.c" 
)

SET(CONFIGS
    "conf/lsm9ds1.json"
)

ADD_LIBRARY(lsm9ds1 SHARED ${SOURCES} ${CONFIGS})
TARGET_LINK_LIBRARIES(lsm9ds1 cjson)
TARGET_LINK_LIBRARIES(lsm9ds1 bcm2835)
SET_TARGET_PROPERTIES(lsm9ds1 PROPERTIES VERSION ${LSM9DS1_VERSION_STRING}
                                          SOVERSION ${LSM9DS1_VERSION_MAJOR})
SET_TARGET_PROPERTIES(lsm9ds1 PROPERTIES PUBLIC_HEADER "${HEADERS}")

######################### Tests ###############################################
OPTION(BUILD_TESTS "Build lsm9ds1 tests" OFF)
if(BUILD_TESTS)
    FILE(GLOB TEST_SOURCES
    	"tests/test.c"
    )
    ADD_EXECUTABLE(lsm9ds1_test ${TEST_SOURCES})
    TARGET_LINK_LIBRARIES(lsm9ds1_test cunit)
    TARGET_LINK_LIBRARIES(lsm9ds1_test lsm9ds1)

    ######################### Tests ###########################################
    INSTALL(TARGETS lsm9ds1_test
        DESTINATION bin
    )
endif(BUILD_TESTS)

######################## Install targets ######################################

########################## C Library ##########################################
INSTALL(TARGETS lsm9ds1
	DESTINATION lib
    PUBLIC_HEADER DESTINATION include    
)

INSTALL(FILES ${CONFIGS}
    DESTINATION etc
)
###############################################################################

######################## RPM Package ##########################################
SET(DEPENDENCIES "libcjson1 >= 1.0-r0")
SET(CPACK_RPM_PACKAGE_AUTOREQ OFF)
IF(CMAKE_BUILD_TYPE MATCHES Debug)
    SET(DEPENDENCIES "${DEPENDENCIES}, libcunit1 >= 2.1+3-r0")
ENDIF(CMAKE_BUILD_TYPE MATCHES Debug)

SET(CPACK_RPM_PACKAGE_REQUIRES ${DEPENDENCIES})
SET(CPACK_RPM_PACKAGE_RELOCATABLE ON)
SET(CPACK_RPM_PACKAGE_ARCHITECTURE "raspberrypi3")
SET(CPACK_PACKAGE_VERSION ${BUILD_VERSION})
SET(CPACK_GENERATOR "RPM;TGZ")
SET(CPACK_PACKAGE_NAME ${PROJECT})
SET(CPACK_PACKAGE_RELEASE 1)
SET(CPACK_PACKAGE_CONTACT "Christopher Jordan-Denny")
SET(CPACK_PACKAGE_VENDOR "Christopher Jordan-Denny")
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Library for the LSM9DS1.")
SET(CPACK_RPM_PACKAGE_LICENSE ${LICENSE})
SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
STRING(CONCAT CPACK_PACKAGE_FILE_NAME 
    "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}"
    "-${CPACK_RPM_PACKAGE_ARCHITECTURE}")
INCLUDE(CPack)
###############################################################################
