#!/bin/sh

# Exit on the first error
set -e

# Setup Environment
rm -rf build

# Build without ros wrapper to catch warnings and errors
sphinx-build -W -b html . build

# Validate if the HTML is valid (i.e. no missing links, images, etc.)
htmlproofer docs/build/ --check-favicon --check-html --check-img-http --check-sri --enforce-https --only-4xx --file-ignore docs/build/html/genindex.html,docs/build/html/search.html,docs/build/html/index-ms g.html,docs/build/search.html --hydra-config '{"max_concurrency": 3}' --url-ignore '#'

# Run
xdg-open ./build/html/index.html
