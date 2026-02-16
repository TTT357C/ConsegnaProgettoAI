#!/bin/bash

# Clean previous build and CMake cache files
rm -rf build
rm -f CMakeCache.txt
rm -rf CMakeFiles
rm -f cmake_install.cmake
mkdir build

# build exec for cpp
echo "Configuring with CMake..."
cmake -B build ./ -DPYTHON=false -DCMAKE_BUILD_TYPE=Release

echo "Building with make (using 2 parallel jobs to avoid memory issues)..."
make -C build -j2

echo "Build complete."


# build exec for python

# cmake -B build ./ -DPYTHON=true -DCMAKE_BUILD_TYPE=Release
# make -C build -j
