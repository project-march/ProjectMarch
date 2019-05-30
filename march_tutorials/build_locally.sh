#!/bin/sh

# Setup Environment
rm -rf build

# Build
rosdoc_lite -o build .

# Run
xdg-open ./build/html/index.html
