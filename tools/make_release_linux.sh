#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

SCONE_SOURCE_DIR="${SCRIPT_DIR}/.."
SCONE_BUILD_DIR="${PWD}/build"
SCONE_INSTALL_DIR="${PWD}/scone-core"

echo ${SCRIPT_DIR}
echo ${SCONE_SOURCE_DIR}

cmake_args=(
    -DCMAKE_BUILD_TYPE="Release"
    -DCMAKE_INSTALL_PREFIX="${SCONE_INSTALL_DIR}"
    -DSCONE_HYFYDY=ON
    -DSCONE_LUA=ON
    -DSCONE_PYTHON=ON
    -DSCONE_CORE_CPACK=ON
)

mkdir -p "${SCONE_BUILD_DIR}"
mkdir -p "${SCONE_INSTALL_DIR}"

# configure cmake
cd "${SCONE_BUILD_DIR}"
cmake ${SCONE_SOURCE_DIR} ${cmake_args[@]}

# build and install
cmake --build . --parallel
cmake --install .

# create package(s)
cpack