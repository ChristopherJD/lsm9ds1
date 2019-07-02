DEBUG_DIR="Debug"
RELEASE_DIR="Release"
DOCUMENTS_DIR="Docs"

build_type=${1,,}

RELEASE=false
DEBUG=false
if [ "release" == "${build_type}" ]; then
	RELEASE=true
	echo "RELEASE Build..."
else
	DEBUG=true
	echo "DEBUG Build..."
fi

if ${DEBUG}; then
	if [ -d "${DEBUG_DIR}" ]; then rm -Rf ${DEBUG_DIR}; fi
	mkdir ${DEBUG_DIR}
	pushd ${DEBUG_DIR}
	cmake -DCMAKE_BUILD_TYPE=Debug ..
fi

if ${RELEASE}; then
	if [ -d "${RELEASE_DIR}" ]; then rm -Rf ${RELEASE_DIR}; fi
	VERSION=$(git tag)
	mkdir ${RELEASE_DIR}
	pushd ${RELEASE_DIR}
	cmake -DBUILD_VERSION=${VERSION} -DBUILD_DOCUMENTATION=ON -DCMAKE_BUILD_TYPE=Release ..
	make doc
	cp -r docs/* ../docs
	make package
fi

make

popd
