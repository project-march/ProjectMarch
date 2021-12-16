#!/bin/sh
# This script is designed to automatically create a python virtual environment with all necessary dependencies.
# Note: This file virtual environment is not meant to run the ros code in, but for your IDE to easily index dependencies.\
# To run the march code with this venv you need to add the options --system-site-packages --symlinks to building,
# however this is not advised and you should use the docker images if you want to set the dependencies to
# a specific version.
# To run this code call `bash create_venv.sh` in a terminal console.
# date: 10-2021
# author: George Vegelien

# Exit on the first error
set -e

CACHE=true

while (("$#")); do
  case "$1" in
  # The default output if someone calls help on the bash script.
  -h | --help)
    echo "This bash script will make a virtual environment in the current folder."
    echo "This will be a folder called '.venv_march'."
    echo "To use this virtual environment call 'source .venv_march/bin/activate'"
    printf "\n -n | --new \t\t\tThis will delete the current environment."
    printf "\n -c | --clean | --no-cache \tThis will make sure that it freshly downloads all dependencies."
    echo ""
    exit 0;
    ;;
  # The new parameter to delete the previous venv. False by default in case of an accidental call.
  -n | --new=) # unsupported flags
    echo "The current virtual environment will be deleted."
    rm -rf .venv_march/
    shift
    ;;
  # Installs all packages with the no-cache param to ensure a clean install.
  -c | --no-cache | --clean)
    CACHE=false
    shift
    ;;
  -* | --*=) # unsupported flags
    echo "Error: Unsupported flag $1" >&2
    exit 1
    ;;
  *) # preserve positional arguments
    echo "Error: Unsupported arguments $1" >&2
    exit 1
    ;;
  esac
done

cd ~/march/

python3 -m venv .venv_march

source ~/march/.venv_march/bin/activate

if [ $CACHE = true ]; then
  pip install wheel==0.37.0
  pip install -r requirements.txt
  pip install -r requirements_colcon.txt
else
  pip install --no-cache wheel==0.37.0
  pip install --no-cache -r requirements.txt
  pip install --no-cache -r requirements_colcon.txt
fi
