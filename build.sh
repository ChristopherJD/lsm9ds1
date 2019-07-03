DEBUG_DIR="Debug"
RELEASE_DIR="Release"
build_type=${1,,}
release_options=${2,,}

function help() {
	echo -e "${0} <Debug | Release[ docs]> "
	echo -e "\tDebug\tCreate a build with debug symbos."
	echo -e "\tRelease\tCreate release build. This includes documentation."
	echo -e "\tDocs\tBuild the doxygen documentation. Can only be used with release"
}

###############################################################################
#	Parse Options
###############################################################################
RELEASE=false
DEBUG=false
DOCS=false
EXTRA_CMAKE_OPTIONS=""
if [ "release" == "${build_type}" ]; then
	RELEASE=true
	echo "RELEASE Build..."

	if [ "docs" == "${release_options}" ]; then
		DOCS=true
		EXTRA_CMAKE_OPTIONS=" -DBUILD_DOCUMENTATION=ON"
		echo "Building docs..."
	fi
elif [ "debug" == "${build_type}" ]; then
	DEBUG=true
	echo "DEBUG Build..."
else
	help
	echo "Uknown Option!"
fi

###############################################################################
#	Run Tasks
###############################################################################
if ${DEBUG}; then
	if [ -d "${DEBUG_DIR}" ]; then rm -Rf ${DEBUG_DIR}; fi
	VERSION=$(git describe --tags)
	mkdir ${DEBUG_DIR}
	pushd ${DEBUG_DIR}
	cmake -DBUILD_VERSION=${VERSION} -DCMAKE_BUILD_TYPE=Debug ..
	make package
	make
	popd
fi

if ${RELEASE}; then
	if [ -d "${RELEASE_DIR}" ]; then rm -Rf ${RELEASE_DIR}; fi
	VERSION=$(git describe --tags --abbrev=0)
	mkdir ${RELEASE_DIR}
	pushd ${RELEASE_DIR}
	cmake ${EXTRA_CMAKE_OPTIONS} -DBUILD_VERSION=${VERSION} -DCMAKE_BUILD_TYPE=Release ..
	make package
	make
	popd
fi

if ${DOCS}; then
	pushd ${RELEASE_DIR}
	make doc
	cp -r docs/* ../docs
	popd
fi
