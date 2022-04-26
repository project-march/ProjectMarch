"""Author: Tuhin Das, MVII"""

import rospy
from .stone_finder import StoneFinder
import pyrealsense2 as rs
from dynamic_reconfigure.server import Server
from march_stone_finder.cfg import parametersConfig


def callback(config, _):
    rospy.loginfo("Updated parameters of stone finder")
    return config


def main():

    rospy.init_node("march_stone_finder", anonymous=True)
    Server(parametersConfig, callback)

    context = rs.context()
    pipelines = [None, None]

    for dev in context.query_devices():
        dev.hardware_reset()
        pipe = rs.pipeline(context)
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
        cfg.enable_device(dev.get_info(rs.camera_info.serial_number))
        pipe.start(cfg)
        pipelines.append(pipe)

        if dev.get_info(rs.camera_info.serial_number) == "109122070820":
            pipelines[0] = pipe
        elif dev.get_info(rs.camera_info.serial_number) == "033322070612":
            pipelines[1] = pipe

    if pipelines[0] is None or pipelines[1] is None:
        rospy.signal_shutdown("Was not able to find two realsense cameras")

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

    rospy.Timer(rospy.Duration(0.010), left_callback, True)
    rospy.Timer(rospy.Duration(0.010), right_callback, True)

    rospy.spin()


if __name__ == "__main__":
    main()
