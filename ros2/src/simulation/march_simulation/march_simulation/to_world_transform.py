from typing import List

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped, Vector3
from gazebo_msgs.srv import GetEntityState, GetModelList

NODE_NAME = 'to_world_transform'
MODEL_NAME = 'exo'
SERVICE_TIMEOUT = 2


class WorldTransformer(Node):
    """The WorldTransformer node gets the entity state of the march model
    continuously, and publishes this information on the /tf topic."""

    def __init__(self):
        super().__init__(NODE_NAME)

        self.get_logger().info("Exoskeleton is mobile")

        self.transform = TransformStamped()
        self.transform.header.frame_id = 'world'
        self.transform.child_frame_id = 'base_link'
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_model_list_client = self.create_client(GetModelList, '/get_model_list')
        self.get_entity_state_client = self.create_client(GetEntityState, '/get_entity_state')

        # set this rate equal to the rate of the joint state publisher, such that the TF frames are updated at the same rate.
        self.rate = self.create_rate(50)

    def start(self):
        while not self.get_model_list_client.wait_for_service(SERVICE_TIMEOUT):
            self.get_logger().info('Waiting for /get_model_list to come online')

        while rclpy.ok() and MODEL_NAME not in self.get_model_list():
            self.get_logger().info(f'Waiting for model {MODEL_NAME} to appear in model list')

        while not self.get_entity_state_client.wait_for_service(SERVICE_TIMEOUT):
            self.get_logger().info('Waiting for /get_entity_state to come online')

        while rclpy.ok():
            result = self.get_entity_state(MODEL_NAME)

            if result.success:
                self.transform.transform.rotation = result.state.pose.orientation
                # Convert Point to Vector3
                position = result.state.pose.position
                self.transform.transform.translation = Vector3(x=position.x, y=position.y, z=position.z)

                self.tf_broadcaster.sendTransform(self.transform)

    def get_model_list(self) -> List[str]:
        """Get the model list from gazebo."""
        future = self.get_model_list_client.call_async(GetModelList.Request())
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result.success:
            return result.model_names
        return []

    def get_entity_state(self, model):
        """Get entity state of march robot in gazebo."""
        future = self.get_entity_state_client.call_async(GetEntityState.Request(name=model))
        rclpy.spin_until_future_complete(self, future)

        return future.result()


def main():
    rclpy.init()
    WorldTransformer().start()