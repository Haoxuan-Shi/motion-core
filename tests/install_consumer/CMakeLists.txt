cmake_minimum_required(VERSION 3.16)
project(motion-core-package-consumer LANGUAGES CXX)

find_package(motion-core CONFIG REQUIRED)

add_executable(package-consumer main.cpp)
target_link_libraries(package-consumer PRIVATE motion_core::motion_core)
target_compile_features(package-consumer PRIVATE cxx_std_17)
