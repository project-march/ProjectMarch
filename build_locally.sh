#!/bin/sh

# Exit on the first error
set -e

pip install pygit2 --user

# Setup Environment
rm -rf build

# Build
rosdoc_lite -o build .

# Validate if the HTML is valid (i.e. no missing links, images, etc.)
htmlproofer ./build --only-4xx --check-html --file-ignore ./build/html/genindex.html,./build/html/search.html,./build/html/index-msg.html --alt-ignore '/.*/' --url-ignore '#'

# Run
xdg-open ./build/html/index.html
