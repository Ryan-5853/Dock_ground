#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${BUILD_DIR:-${project_root}/build}"
build_jobs="${BUILD_JOBS:-2}"

cmake \
    -S "${project_root}" \
    -B "${build_dir}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE:-RelWithDebInfo}"
cmake --build "${build_dir}" -j "${build_jobs}"
ctest --test-dir "${build_dir}" --output-on-failure
npm --prefix "${project_root}/frontend" run build
