#!/bin/bash

# Create build directory if it doesn't exist
if [ ! -d "build" ]; then
    mkdir build
fi

# Change to build directory
cd build

# Run cmake
cmake ..

# Build the project
make

echo "Build completed!"