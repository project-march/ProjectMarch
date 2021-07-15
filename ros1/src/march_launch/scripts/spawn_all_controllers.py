#!/usr/bin/env python3
import rospy
from roslaunch.core import Node
from roslaunch.scriptapi import ROSLaunch

NAMESPACE = "march"
CONTROLLER_NAMESPACE = "controller"


def main():
    """This script looks into the ros parameter server and spawns all controller it can find.

    Usage: launch from a launch file with the first argument the namespace to find all controllers in.
    """
    try:
        rospy.init_node("spawn_all_controllers")
    except rospy.ROSInitException:
        return

    controller_config = rospy.get_param(f"/{NAMESPACE}/{CONTROLLER_NAMESPACE}")

    args = "spawn"
    for controller in controller_config:
        args += f" {CONTROLLER_NAMESPACE}/{controller}"

    node = Node(
        package="controller_manager",
        node_type="controller_manager",
        namespace=NAMESPACE,
        name="controller_spawner",
        args=args,
    )
    launch = ROSLaunch()
    launch.start()

    launch.launch(node)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
