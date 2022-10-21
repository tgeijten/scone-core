#!/usr/bin/env bash

set -xeuo pipefail
script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

export CMAKE_VERBOSE_MAKEFILE="${CMAKE_VERBOSE_MAKEFILE:-False}"

source "${script_dir}/build_config"

# ${script_dir}/linux_1_get-external-dependencies
# ${script_dir}/unix_2b_build-simbody-fix
# ${script_dir}/unix_2c_build-opensim3-fix
${script_dir}/unix_2d_build-scone-core-hfd

# install and package
cd "${SCONE_BUILD_DIR}"
cmake --install .
cpack
