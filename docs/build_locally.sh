#!/bin/sh

# Exit on the first error
set -e

# Setup Environment
rm -rf build

# shellcheck disable=SC2112
function readmes_to_sphinx ()
{
    echo "Converting READMEs to reStructuredText..."
    echo "Create empty directory $2"
    rm -rf "$2"
    mkdir -p "$2"

    echo "Find all relevant README.md files in main repository"
    INPUT_FILE=$(mktemp)
    # shellcheck disable=SC2086 # Because we want it to be seperated as loose strings.
    find $1 -name "README.md" | grep -v "libraries" | grep '^' | xargs readlink -f > "$INPUT_FILE"

    # Extract the package name by removing the "README.md" part and by removing
    # everything before the latest "/"
    # Append `.rst` to the end of the name

    # For example 
    # /march/ros2/src/hardware/interface/march_smartglasses_bridge/README.md
    # becomes
    # march_smartglasses_bridge
    echo "Extract package name from README files"
    OUTPUT_DIR=$(readlink -f $2)
    OUTPUT_FILE=$(mktemp)
    cat $INPUT_FILE | sed "s#/README.md##" \
                 | sed "s#.*/##" \
                 | sed "s#^#$OUTPUT_DIR/#" > "$OUTPUT_FILE"

    # Read the contents of the files into arrays
    readarray -t INPUT < $INPUT_FILE
    readarray -t OUTPUT < $OUTPUT_FILE

    CURRENT_DIR=$(pwd)
    # Loop through the indices of the arrays
    for ((i=0; i < ${#INPUT[@]}; i++))
    do
        # Transform each markdown file to a reStructuredText file with pandoc
        # It extracts certain files like images from the relative location of the README
        # and stores a copy close to the converted file.
        cd $(dirname ${INPUT[i]})
        STATIC_DIR=${OUTPUT[i]}/static
        mkdir -p $STATIC_DIR
        echo "Converting ${INPUT[i]} to ${OUTPUT[i]}/README.rst"
        pandoc -f commonmark -t rst --fail-if-warning --extract-media "$STATIC_DIR" ${INPUT[i]} \
            | sed "s#${OUTPUT[i]}/##" > ${OUTPUT[i]}/README.rst
    done
    cd "$CURRENT_DIR"

    # Remove the temporary files
    rm $INPUT_FILE
    rm $OUTPUT_FILE

    echo "Successfully converted READMEs to reStructuredText!"
}

readmes_to_sphinx "doc/march_packages/"
#readmes_to_sphinx "../ros2/src" "doc/march_packages/"

# Build the sphinx documentation while catching warnings and errors
sphinx-build -W -b html . build
