import rclpy
from rclpy.node import Node

from march_shared_msgs.msg import StateEstimatorVisualization

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D, axes3d

class StateEstimatorVisualizerNode(Node):
    def __init__(self):
        super().__init__('state_estimator_visualizer_node')
        self.subscription = self.create_subscription(
            StateEstimatorVisualization,
            'state_estimator/node_positions',
            self.listener_callback,
            1)
        self.subscription  # prevent unused variable warning
        self.figure = plt.figure()
        self.ax = Axes3D(self.figure)

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg)
        nodes = {
            name: {
                'pose': pose, 
                'parent': parent,
            } 
            for name, pose, parent in zip(msg.node_names, msg.node_poses, msg.parent_node_names)
        }
        # names = [name for name in msg.node_names]
        # points = np.array([[pose.position.x, pose.position.y, pose.position.z] for pose in msg.node_poses])
        self.plot(nodes)

    def plot(self, nodes):
        # ax.scatter(points[:,0], points[:,1], points[:,2])
        # for i, name in enumerate(names):
        #     ax.text(points[i,0], points[i,1], points[i,2], name)
        for name, item in nodes.items():
            # ax.scatter(item['pose'].position.x, item['pose'].position.y, item['pose'].position.z)
            if item['parent'] != 'none':
                self.ax.plot(
                    [nodes[item['parent']]['pose'].position.x, item['pose'].position.x], 
                    [nodes[item['parent']]['pose'].position.y, item['pose'].position.y], 
                    [nodes[item['parent']]['pose'].position.z, item['pose'].position.z],
                    c='red'
                )
            self.ax.scatter(item['pose'].position.x, item['pose'].position.y, item['pose'].position.z, c='black')
            self.ax.text(item['pose'].position.x, item['pose'].position.y, item['pose'].position.z, name, alpha=0.5)
        self.ax.set_xlim(-0.5, 0.5)
        self.ax.set_ylim(-0.5, 0.5)
        self.ax.set_zlim(-1.0, 0.0)
        self.figure.canvas.draw()
        plt.pause(1e-9)
        self.figure.canvas.flush_events()
        self.ax.clear()

def main(args=None):
    rclpy.init(args=args)

    state_estimator_visualizer_node = StateEstimatorVisualizerNode()

    rclpy.spin(state_estimator_visualizer_node)

    state_estimator_visualizer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()