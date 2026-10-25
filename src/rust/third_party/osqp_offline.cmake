# CMake toolchain file for osqp-sys's OSQP C build (f1tenth_mpc_core): QDLDL
# from third_party/qdldl, never from the network. See README.md here.
#
# osqp-sys 1.0.1's build.rs builds OSQP C 1.0.0 with the `cmake` crate, and
# OSQP's algebra/_common/lin_sys/qdldl/qdldl.cmake declares QDLDL with
# FetchContent (GIT_REPOSITORY https://github.com/osqp/qdldl.git, GIT_TAG
# v0.1.8): every clean build cloned it. build.rs offers no hook, but the
# `cmake` crate passes CMAKE_TOOLCHAIN_FILE from the target-specific
# environment variable CMAKE_TOOLCHAIN_FILE_<target> (../build_env.sh sets
# it). Plain CMAKE_TOOLCHAIN_FILE is avoided on purpose: CMake >= 3.21 reads
# that one itself, so every CMake package of a colcon build would load this
# file.
#
# Nothing here changes how OSQP or QDLDL compile. One side effect of using a
# toolchain file: the `cmake` crate then leaves CMAKE_C_COMPILER to CMake
# (it passes its absolute path of `cc` otherwise); CMake finds the same
# /usr/bin/cc (CC, if set, is honoured either way). CMAKE_C_FLAGS and
# CMAKE_BUILD_TYPE are still passed by the crate (output/rust_main/r0b.md
# compares the two CMakeCache.txt files).
#
# This file is also read for CMake's try_compile projects: keep it to plain
# set() calls.

set(_f1t_qdldl "${CMAKE_CURRENT_LIST_DIR}/qdldl")
if(NOT EXISTS "${_f1t_qdldl}/CMakeLists.txt")
  message(FATAL_ERROR
    "osqp_offline.cmake: vendored QDLDL missing at ${_f1t_qdldl} "
    "(src/rust/third_party/README.md)")
endif()

# FetchContent uses this directory as qdldl's source: no download, no
# _deps/qdldl-subbuild. The binary dir stays _deps/qdldl-build in the build
# tree (QDLDL writes nothing into its source dir).
set(FETCHCONTENT_SOURCE_DIR_QDLDL "${_f1t_qdldl}" CACHE PATH
  "QDLDL v0.1.8, vendored (src/rust/third_party/qdldl)" FORCE)
# Any other FetchContent download fails instead of reaching the network.
set(FETCHCONTENT_FULLY_DISCONNECTED ON CACHE BOOL
  "No FetchContent downloads (src/rust/third_party/osqp_offline.cmake)" FORCE)
unset(_f1t_qdldl)
