DEBUG_DIR="Debug"
RELEASE_DIR="Release"
DOCUMENTS_DIR="Docs"

if [ -d "${DEBUG_DIR}" ]; then rm -Rf ${DEBUG_DIR}; fi
mkdir Debug

pushd Debug
cmake -DCMAKE_BUILD_TYPE=Debug ..
make
