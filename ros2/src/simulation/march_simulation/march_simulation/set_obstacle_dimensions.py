"""Author: MVI."""
from threading import Event
import rclpy
from rclpy.task import Future
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from march_shared_msgs.srv import SetObstacleSize
from gazebo_msgs.srv import SpawnEntity, DeleteEntity, GetModelList
import xacro

# All obstacles that have a macro file which allows for changing the dimension
# this means there must be an <name>_macro.xacro in the obstacles directory
# which accepts the parameters length, width and height with default values


RESIZABLE_OBSTACLES = ["bench"]

NODE_NAME = "set_obstacle_node"


class ObstacleDimensionSetter(Node):
    """Class to resize the obstacles loaded in gazebo."""

    def __init__(self):
        super().__init__(NODE_NAME)

        self.spawn_entity_client = self.create_client(SpawnEntity, "/spawn_entity")
        self.delete_entity_client = self.create_client(DeleteEntity, "/delete_entity")
        self.get_model_list_client = self.create_client(GetModelList, "/get_model_list")

        self.get_model_list_event = Event()
        self.spawn_entity_event = Event()
        self.delete_entity_event = Event()

        self.create_service(
            SetObstacleSize, "/march/set_obstacle_size", self.set_size_callback, callback_group=ReentrantCallbackGroup()
        )

        self.models = []

    def set_size_callback(self, request, response):
        """Callback function for the service /march/set_obstacle_size, to resize the loaded in obstacles."""
        if request.obstacle_name not in RESIZABLE_OBSTACLES:
            response.success = False
        else:
            self.set_size(
                name=request.obstacle_name,
                length=request.new_length,
                width=request.new_width,
                height=request.new_height,
            )
            response.success = True
        return response

    def set_size(self, name: str, length: float = 0, width: float = 0, height: float = 0):
        """Set the size of an obstacle.

        If the obstacle is a new obstacle it is spawned using the provided dimensions.
        If the obstacle already exists, it is first removed from gazebo.

        Args:
            name (str): Name of the obstacle.
            length (float): Length of the obstacle.
            width (float): Width of the obstacle.
            height (float): Height of the obstacle.
        """
        length = 'length="{length}"'.format(length=length) if length != 0 else ""
        width = 'width="{width}"'.format(width=width) if width != 0 else ""
        height = 'height="{height}"'.format(height=height) if height != 0 else ""

        doc = xacro.parse(
            """<?xml version="1.0"?>
                <robot name="{name}" xmlns:xacro="http://www.ros.org/wiki/xacro">
                    <xacro:include filename="$(find march_simulation)/obstacles/{name}_macro.xacro"/>
                    <xacro:{name} {length} {width} {height}/>
                </robot>
                """.format(
                name=name, length=length, width=width, height=height
            ),
            None,
        )
        xacro.process_doc(doc)
        new_obstacle = doc.toprettyxml(indent="  ")

        self.get_model_list()

        if name in self.models:
            self.delete_entity(name)

        self.spawn_entity(name, new_obstacle)

    def get_model_list(self):
        """Get the model list from gazebo."""
        self.get_model_list_event.clear()
        future = self.get_model_list_client.call_async(GetModelList.Request())
        future.add_done_callback(self.get_model_list_cb)
        self.get_model_list_event.wait()

    def get_model_list_cb(self, future: Future):
        """Callback for when getting the model list is done."""
        result = future.result()
        if result.success:
            self.models = result.model_names
        self.get_model_list_event.set()

    def delete_entity(self, name: str):
        """Delete an entity from gazebo."""
        self.delete_entity_event.clear()
        future = self.delete_entity_client.call_async(DeleteEntity.Request(name=name))
        future.add_done_callback(self.delete_entity_cb)
        self.delete_entity_event.wait()

    def delete_entity_cb(self, future: Future):
        """Callback for when deleting an entity is done."""
        result = future.result()
        if not result.success:
            self.get_logger().fatal("Unable to delete obstacle")
        self.delete_entity_event.set()

    def spawn_entity(self, name: str, xml: str):
        """Spawn an entity in gazebo."""
        self.spawn_entity_event.clear()
        future = self.spawn_entity_client.call_async(SpawnEntity.Request(name=name, xml=xml))
        future.add_done_callback(self.spawn_entity_cb)
        self.spawn_entity_event.wait()

    def spawn_entity_cb(self, future: Future):
        """Callback for when spawning an entity is done."""
        result = future.result()
        if not result.success:
            self.get_logger().fatal("Unable to spawn obstacle")
        self.spawn_entity_event.set()


def main():
    """The entry script function."""
    rclpy.init()
    executor = MultiThreadedExecutor()
    node = ObstacleDimensionSetter()
    rclpy.spin(node, executor)


if __name__ == "__main__":
    main()
