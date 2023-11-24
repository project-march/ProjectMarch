#!/bin/bash

# Create a named pipe
mkfifo /tmp/my_fifo

# Open gnome-terminal and read from the named pipe
gnome-terminal -- bash -c 'cat /tmp/my_fifo'

# Remove the named pipe when done
rm /tmp/my_fifo