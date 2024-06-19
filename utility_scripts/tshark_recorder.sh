#!/bin/bash
# Check if the user provided the necessary arguments
if [ "$#" -lt 2 ]; then
  echo "Usage: $0 <interface> <foreground_process_command> [arguments...]"
  exit 1
fi
# Extract the interface name from the first argument
interface=$1
shift
# Generate a timestamp for the capture filename
timestamp=$(date +"%Y%m%d_%H%M_%S")
capture_file="$HOME/tshark-captures/ethercat_capture_${interface}_${timestamp}.pcap"
# Start the user-specified foreground process and capture its PID
"$@" &
fg_pid=$!
# Start tshark in the background and capture its PID
tshark -i "$interface" -w "$capture_file" &
bg_pid=$!
# Wait for the foreground process to complete
wait $fg_pid
# Kill the tshark process once the foreground process completes
sudo kill $bg_pid
echo "tshark capture stopped because the foreground process has ended."
echo "Capture file saved as $capture_file."
