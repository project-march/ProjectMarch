#!/bin/sh

# Exit on the first error
set -e

cd ~/march/

rm -rf .venv_march/

python3 -m venv .venv_march

source .venv_march/bin/activate

pip install wheel==0.37.0

pip install -r requirements.txt

pip install -r requirements_colcon.txt
