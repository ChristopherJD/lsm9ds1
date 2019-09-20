VERSION:=$(shell git describe --tags --abbrev=0)
DEBUG_DIR:="Debug"
RELEASE_DIR:="Release"

all: generate_make package

debug: generate_debug_make debug_package

generate_debug_make:
	mkdir ${DEBUG_DIR}; pushd ${DEBUG_DIR}; cmake -DBUILD_VERSION=${VERSION} -DBUILD_TESTS=ON -DCMAKE_BUILD_TYPE=Debug ..

generate_make:
	mkdir ${RELEASE_DIR}; pushd ${RELEASE_DIR}; cmake -DBUILD_VERSION=${VERSION} -DBUILD_DOCUMENTATION=ON -DCMAKE_BUILD_TYPE=Release ..

debug_package:
	${MAKE} -C ${DEBUG_DIR} package

package:
	${MAKE} -C ${RELEASE_DIR} package

clean:
	-rm -rf ${RELEASE_DIR}

debug_clean:
	-rm -rf ${DEBUG_DIR}