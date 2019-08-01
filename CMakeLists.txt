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
PROJECT(liblsm9ds1)
SET(PROJECT liblsm9ds1)
SET(LICENSE "GPLV3")
###############################################################################

########################### Build and include settings ########################
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DDEBUG=1 -g")
STRING(CONCAT CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Wall -pedantic")

INCLUDE_DIRECTORIES(
    inc
)

LINK_DIRECTORIES(
    ${LINK_DIRECTORIES}
)

########################## C Library ##########################################
SET(SOURCES 
    "src/lsm9ds1.c" 
    "src/lsm9ds1_common.c"
    "src/lsm9ds1_bus.c"
    "src/lsm9ds1_accel.c"
    "src/lsm9ds1_gyro.c"
    "src/lsm9ds1_mag.c"
    "src/lsm9ds1_temp.c" 
    "src/lsm9ds1_config.c" 
)

FILE(GLOB CONFIGS
    "conf/lsm9ds1.json"
)

ADD_LIBRARY(lsm9ds1 SHARED ${SOURCES} ${CONFIGS})
TARGET_LINK_LIBRARIES(lsm9ds1 cjson)
SET_TARGET_PROPERTIES(lsm9ds1 PROPERTIES VERSION ${LSM9DS1_VERSION_STRING}
                                          SOVERSION ${LSM9DS1_VERSION_MAJOR})

######################### Tests ###############################################
FILE(GLOB TEST_SOURCES
	"tests/test.c"
)
ADD_EXECUTABLE(lsm9ds1_test ${TEST_SOURCES})
TARGET_LINK_LIBRARIES(lsm9ds1_test cunit)
TARGET_LINK_LIBRARIES(lsm9ds1_test lsm9ds1)

######################## Install targets ######################################

########################## C Library ##########################################
INSTALL(TARGETS lsm9ds1
	DESTINATION usr/lib/
    COMPONENT binaries
)

INSTALL(FILES ${CONFIGS}
    DESTINATION etc/
    COMPONENT config
)

######################### Tests ###############################################
INSTALL(TARGETS lsm9ds1_test
	DESTINATION usr/bin
)
###############################################################################

######################## RPM Package ##########################################
# Unit test picks up lsm9ds1 lib so I need to manually do this
SET(CPACK_RPM_PACKAGE_AUTOREQ no)
SET(CPACK_RPM_PACKAGE_REQUIRES "wiringpi")
SET(CPACK_RPM_PACKAGE_RELOCATABLE ON)
SET(DESTDIR /)
SET(CPACK_RPM_PACKAGE_ARCHITECTURE "raspberrypi3")
SET(CPACK_PACKAGE_VERSION ${BUILD_VERSION})
SET(CPACK_GENERATOR "RPM;TGZ")
SET(CPACK_PACKAGE_NAME ${PROJECT})
SET(CPACK_PACKAGE_RELEASE 1)
SET(CPACK_PACKAGE_CONTACT "Christopher Jordan-Denny")
SET(CPACK_PACKAGE_VENDOR "Christopher Jordan-Denny")
SET(CPACK_PACKAGING_INSTALL_PREFIX ${DESTDIR})
STRING(CONCAT CPACK_PACKAGE_FILE_NAME 
    "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}"
    "-${CPACK_RPM_PACKAGE_ARCHITECTURE}")
INCLUDE(CPack)
###############################################################################
