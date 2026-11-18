# shellcheck shell=bash
# Build environment for the Rust packages in src/rust: source it (bash or zsh)
# in the shell that runs `colcon build` (with the ament_cargo packages) or
# cargo in src/rust, after the ROS setup files:
#   source /opt/ros/jazzy/setup.bash
#   source <repo>/src/rust/build_env.sh
#
# It points the `cmake` crate at third_party/osqp_offline.cmake through
# CMAKE_TOOLCHAIN_FILE_<host triple>, so osqp-sys's OSQP build takes QDLDL
# from third_party/qdldl and never fetches it (third_party/README.md). The
# target-specific name is read by the `cmake` crate only; CMake itself reads
# only plain CMAKE_TOOLCHAIN_FILE, so the colcon build's CMake packages are
# not affected.
#
# Without it the build still works where github.com is reachable (OSQP's
# CMake clones QDLDL v0.1.8, the same commit), but not offline.
#
# cargo does not rebuild osqp-sys when this variable changes (the `cmake`
# crate does not declare it); it matters for a clean build, which is what the
# image does.

# This file's directory, whichever shell sources it. The zsh expansion is
# only ever evaluated by zsh (bash never runs that branch).
if [ -n "${ZSH_VERSION:-}" ]; then
  # shellcheck disable=SC2296
  _f1t_rust_dir=${${(%):-%x}:A:h}
else
  _f1t_rust_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
fi

_f1t_host=$(rustc -vV 2>/dev/null | sed -n 's/^host: //p')
if [ -z "${_f1t_host}" ]; then
  echo "build_env.sh: rustc not found on PATH (needed for the host triple)" >&2
  unset _f1t_rust_dir _f1t_host
  return 1
fi
# The `cmake` crate tries CMAKE_TOOLCHAIN_FILE_<triple> and then
# CMAKE_TOOLCHAIN_FILE_<triple with _ for ->; only the second is a valid
# shell name (CMAKE_TOOLCHAIN_FILE_x86_64_unknown_linux_gnu on linus,
# CMAKE_TOOLCHAIN_FILE_aarch64_unknown_linux_gnu on the Thor and the Orin).
export "CMAKE_TOOLCHAIN_FILE_$(printf '%s' "${_f1t_host}" | tr '-' '_')=${_f1t_rust_dir}/third_party/osqp_offline.cmake"
unset _f1t_rust_dir _f1t_host
