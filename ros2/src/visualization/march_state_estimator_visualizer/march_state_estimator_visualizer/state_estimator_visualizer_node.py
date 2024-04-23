import rclpy
from rclpy.node import Node

from march_shared_msgs.msg import StateEstimatorVisualization

import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D, axes3d

class StateEstimatorVisualizerNode(Node):
    def __init__(self):
        super().__init__('state_estimator_visualizer_node')
        # self.timer = self.create_timer(1e-2, self.timer_callback)
        self.subscription = self.create_subscription(
            StateEstimatorVisualization,
            'state_estimation/visualization',
            self.listener_callback,
            1)
        self.subscription  # prevent unused variable warning
        self.figure = plt.figure()
        self.ax = Axes3D(self.figure)
        self.nodes = None

    def listener_callback(self, msg):
        # for name, pose, parent in zip(msg.node_names, msg.node_poses, msg.parent_node_names):
        #     self.get_logger().info('I heard: "%s" "%s" "%s"' % (name, pose, parent))
        self.nodes = {
            name: {
                'pose': pose, 
                'parent': parent,
            } 
            for name, pose, parent in zip(msg.node_names, msg.node_poses, msg.parent_node_names)
        }
        # names = [name for name in msg.node_names]
        # points = np.array([[pose.position.x, pose.position.y, pose.position.z] for pose in msg.node_poses])
        self.plot(self.nodes)

    def timer_callback(self):
        if self.nodes is not None:
            self.plot(self.nodes)

    def plot(self, nodes):
        # ax.scatter(points[:,0], points[:,1], points[:,2])
        # for i, name in enumerate(names):
        #     ax.text(points[i,0], points[i,1], points[i,2], name)

        backpack_position = nodes['backpack']['pose'].position

        for name, item in nodes.items():
            # ax.scatter(item['pose'].position.x, item['pose'].position.y, item['pose'].position.z)
            if item['parent'] != 'none':
                self.ax.plot(
                    [nodes[item['parent']]['pose'].position.x, item['pose'].position.x], 
                    [nodes[item['parent']]['pose'].position.y, item['pose'].position.y], 
                    [nodes[item['parent']]['pose'].position.z, item['pose'].position.z],
                    c='gray',
                    alpha=0.7,
                )

            q = np.array([
                nodes[name]['pose'].orientation.x, 
                nodes[name]['pose'].orientation.y, 
                nodes[name]['pose'].orientation.z,
                nodes[name]['pose'].orientation.w])
            q = q / np.linalg.norm(q)
            R = Rotation.from_quat(q).as_matrix()
            coord_x = item['pose'].position.x + R[0, :] * 0.1
            coord_y = item['pose'].position.y + R[1, :] * 0.1
            coord_z = item['pose'].position.z + R[2, :] * 0.1

            self.ax.plot([item['pose'].position.x, coord_x[0]], [item['pose'].position.y, coord_y[0]], [item['pose'].position.z, coord_z[0]], c='red')
            self.ax.plot([item['pose'].position.x, coord_x[1]], [item['pose'].position.y, coord_y[1]], [item['pose'].position.z, coord_z[1]], c='green')
            self.ax.plot([item['pose'].position.x, coord_x[2]], [item['pose'].position.y, coord_y[2]], [item['pose'].position.z, coord_z[2]], c='blue')

            # euler = Rotation.from_quat(q).as_euler('zyx', degrees=False)
            # self.ax.quiver(item['pose'].position.x, item['pose'].position.y, item['pose'].position.z,
            #                 euler[0], 0, 0, length=0.1, normalize=True, color='red')
            # self.ax.quiver(item['pose'].position.x, item['pose'].position.y, item['pose'].position.z,
            #                 0, euler[1], 0, length=0.1, normalize=True, color='green')
            # self.ax.quiver(item['pose'].position.x, item['pose'].position.y, item['pose'].position.z,
            #                 0, 0, euler[2], length=0.1, normalize=True, color='blue')
                
            self.ax.scatter(item['pose'].position.x, item['pose'].position.y, item['pose'].position.z, c='black', alpha=0.7)
            self.ax.text(item['pose'].position.x, item['pose'].position.y, item['pose'].position.z, name, alpha=0.5)

        self.ax.set_xlim(-0.5 + backpack_position.x, 0.5 + backpack_position.x)
        self.ax.set_ylim(-0.5 + backpack_position.y, 0.5 + backpack_position.y)
        self.ax.set_zlim(-1.0 + backpack_position.z, 0.0 + backpack_position.z)
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