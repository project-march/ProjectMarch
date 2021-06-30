#!/bin/sh

# Exit on the first error
set -e

# Setup Environment
rm -rf build

function readmes_to_sphinx ()
{
    echo "Converting READMEs to reStructuredText..."
    # Create the given directory if it does not yet exist
    rm -rf $2
    mkdir -p $2

    # List all README.md files
    INPUT=$(mktemp)
    find $1 -name "README.md" | grep -v "libraries" | grep '^' > $INPUT

    # Extract the package name by removing the "README.md" part and by removing
    # everything before "/march"
    # Add the second parameter as before the package name
    # Append `.rst` to the end of the name
    # Prepend "-o" as the pandoc output parameter to the name

    # For example, if called with "foxy" as a parameter then
    # ../ros2/src/hardware/interface/march_smartglasses_bridge/README.md
    # becomes
    # foxy/march_smartglasses_bridge.rst
    OUTPUT=$(mktemp)
    cat $INPUT | sed "s#/README.md##" \
                 | sed "s#.*/##" \
                 | sed "s#^#$2#" \
                 | sed "s/$/\.rst/" \
                 | sed "s/^/-o /" > $OUTPUT

    # Transform each markdown file to a reStructuredText file with pandoc
    # This command will become something like
    # pandoc ../ros2/src/hardware/interface/march_smartglasses_bridge/README.md
    #        -o march_smartglasses_bridge.rst
    # for every found README.md file
    paste -d ' ' $INPUT $OUTPUT | xargs -I{} -- sh -c "pandoc {}"

    # Remove the temporary files
    rm $INPUT
    rm $OUTPUT
    echo "Succesfully converted READMEs to reStructuredText!"
}

readmes_to_sphinx "../ros1/src ../ros2/src" "doc/march_packages/readme/"

# Build the sphinx documentation while catching warnings and errors
sphinx-build -W -b html . build
