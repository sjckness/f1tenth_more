cmake_minimum_required(VERSION 3.8)
project(corridor_perception_c C)

find_package(ament_cmake REQUIRED)

# -O3 and NOTHING ELSE that changes arithmetic. No -ffast-math: this library
# has to agree with numpy to 1e-4 and fast-math reassociates sums and drops
# the inf/NaN handling the no-return convention depends on. No -march: the
# same object has to build and behave identically on the Orin and on Thor,
# and an -march=native build is neither portable nor reproducible.
add_compile_options(-O3 -fno-fast-math -std=c11 -Wall -Wextra -Wpedantic)
# M_PI is not ISO C: under -std=c11 glibc exposes it only with _DEFAULT_SOURCE
# (Jazzy/Ubuntu 24.04 build: 'M_PI undeclared'). The define widens the headers
# only; -std=c11 stays, so FP contraction stays off (ISO mode) on aarch64 too.
add_compile_definitions(_DEFAULT_SOURCE)

add_library(extract_lines SHARED src/extract_lines.c)
target_include_directories(extract_lines PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(extract_lines PROPERTIES C_STANDARD 11 C_EXTENSIONS ON)
target_link_libraries(extract_lines m)

# Into lib/ so ament_index + get_package_prefix finds it as lib/libextract_lines.so.
install(TARGETS extract_lines
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_package()
