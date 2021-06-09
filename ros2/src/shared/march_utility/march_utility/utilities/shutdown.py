import os
import signal
import subprocess  # noqa: S404
import re

# Find the process id (pid) of a process with the following regex
PID_REGEX = "([0-9]{4,}).{10}[0-9]{2}:[0-9]{2}:[0-9]{2} "

# Shutdown the following processes to shut down the entire system
# roslaunch and roscore refer to ROS1 processes
# parameter_bridge and dynamic_bridge refer to bridge processes
# march_gait_selection is a required node in ROS2, ending this process shuts all ROS2 processes down.
ROS_PROCESSES = [
    "roslaunch",
    "roscore",
    "parameter_bridge",
    "dynamic_bridge",
    "march_gait_selection",
]

# ROS processes have a maximum name length of 15 characters
ROS_PROCESS_NAME_LENGTH = 15


def shutdown_system():
    """Shutdown ROS1, the bridge and ROS2."""
    result = subprocess.run(["/usr/bin/ps", "-A"], capture_output=True)  # noqa: S603
    processes = result.stdout.decode("utf-8")

    for process_name in ROS_PROCESSES:
        pattern = PID_REGEX + process_name[:ROS_PROCESS_NAME_LENGTH]
        matches = re.findall(pattern, processes)
        if len(matches) > 0:
            pid = int(matches[0])
            try:
                os.kill(pid, signal.SIGTERM)
            except PermissionError:
                continue
