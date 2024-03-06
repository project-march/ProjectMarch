#!/bin/bash

# Change directory to the specified location
cd /home/pmarch/rosbags2 || exit

# Find the most recently modified directory
most_recent_dir=$(ls -td -- */ | head -n 1)

# Check if there are any directories
if [ -z "$most_recent_dir" ]; then
  echo "No directories found."
  exit 1
fi

# Change directory to the most recently modified directory
cd "$most_recent_dir" || exit

# Find a file with .db3 extension
input_file=$(find . -maxdepth 1 -type f -name "*.db3" | head -n 1)

# Check if input file exists
if [ -z "$input_file" ]; then
  echo "No .db3 file found."
  exit 1
fi

# Extract the file name without extension
filename=$(basename "$input_file" .db3)

# Run mcap convert
echo "Converting $input_file to $filename.mcap"
mcap convert "$input_file" "$filename.mcap"
