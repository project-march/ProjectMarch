#!/bin/sh

# Exit on the first error
set -e

cd ~/march/

python3 -m venv .venv_march

source .venv_march/bin/activate

CACHE=true

while (("$#")); do
  case "$1" in
  # The default output if someone calls help on the bash script.
  -h | --help)
    echo "This bash script will make a virtual environment in the current folder."
    echo "This will be a folder called '.venv_march'."
    printf "\n -n | --new \t\t\tThis will delete the current environment."
    printf "\n -c | --clean | --no-cache \tThis will make sure that it freshly downloads all dependencies."
    echo ""
    shift
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

if [ $CACHE = true ]; then
  pip install wheel==0.37.0
  pip install -r requirements.txt
  pip install -r requirements_colcon.txt
else
  pip install --no-cache wheel==0.37.0
  pip install -r --no-cache requirements.txt
  pip install -r --no-cache requirements_colcon.txt
fi
