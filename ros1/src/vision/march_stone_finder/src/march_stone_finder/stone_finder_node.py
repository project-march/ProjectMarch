"""Author: Tuhin Das, MVII."""

import rospy
import time
from .stone_finder import StoneFinder
import pyrealsense2 as rs
from dynamic_reconfigure.server import Server
from march_stone_finder.cfg import parametersConfig


def callback(config, _):
    """Notify when parameters are updated."""
    rospy.loginfo("Updated parameters of stone finder")
    return config


def main():
    """Connect to realsense devices and run stone finding algorithm on the received frames."""
    rospy.init_node("march_stone_finder")
    Server(parametersConfig, callback)

    context = rs.context()
    pipelines = [None, None]
    serial_numbers = ["944622074337", "944622071535"]
    serial_numbers = ["109122070820"]

    # Perform hardware resets on cameras
    while True:
        try:
            for dev in context.query_devices():
                dev.hardware_reset()
        except RuntimeError as e:
            rospy.logwarn(f"[march_stone_finder] Could not perform hardware reset. ({e}) Retrying...")
            continue
        break

    # Start frame pipelines for left and right cameras
    for index, serial in enumerate(serial_numbers):
        pipe = rs.pipeline()
        config = rs.config()
        while True:
            side = "left" if index == 0 else "right"
            try:
                config.enable_device(serial)
                config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
                config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
                pipe.start(config)
            except RuntimeError as e:
                rospy.logwarn(f"[march_stone_finder] Error while initializing {side} RealSense ({e})")
                time.sleep(1)
                continue

            rospy.loginfo(f"[march_stone_finder] \033[1;36m{side} RealSense connected ({serial}) \033[0m")
            pipelines[index] = pipe
            break

    left_stone_finder = StoneFinder("left")
    right_stone_finder = StoneFinder("right")

    def left_callback(_):
        while not rospy.is_shutdown():
            left_frame = pipelines[0].wait_for_frames()
            left_stone_finder.find_points(left_frame)

    def right_callback(_):
        while not rospy.is_shutdown():
            right_frame = pipelines[1].wait_for_frames()
            right_stone_finder.find_points(right_frame)

    rospy.Timer(rospy.Duration(1.0), left_callback, oneshot=True)
    # rospy.Timer(rospy.Duration(1.0), right_callback, oneshot=True)

    rospy.spin()


if __name__ == "__main__":
    main()
