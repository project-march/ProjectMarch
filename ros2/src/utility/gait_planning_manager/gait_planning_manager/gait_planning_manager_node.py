import rclpy 
from rclpy.node import Node 
from rclpy.executors import MultiThreadedExecutor
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import GetState, ChangeState
from march_shared_msgs.msg import ExoMode 


COLOR_GREEN = "\033[32m"
RESET = "\033[0m"
COLOR_MAGENTA = "\033[35m"

class ServiceClient(Node): 
    def __init__(self): 
        super().__init__('gait_planning_manager_node')

        self.angles_client_change_state = self.create_client(ChangeState, 'gait_planning_angles_node/change_state')
        self.cartesian_client_change_state = self.create_client(ChangeState, 'gait_planning_cartesian_node/change_state')
        
        self.mode_subscriber = self.create_subscription(
            ExoMode, 'current_mode', self.mode_callback, 10)
        self.gaitplanning_mode_publisher = self.create_publisher(
            ExoMode, 'gait_planning_mode', 10)

        self.angles_active = True
        self.cartesian_active = True
        self.future = True

        if not self.change_angles_state(ChangeState.Request().transition.TRANSITION_CONFIGURE):
            self.get_logger().info(COLOR_MAGENTA + 'Angles not able to configure' + RESET)
        if not self.change_cartesian_state(ChangeState.Request().transition.TRANSITION_CONFIGURE): 
            self.get_logger().info(COLOR_MAGENTA + 'Cartesian not able to configure' + RESET)
        if not self.change_angles_state(ChangeState.Request().transition.TRANSITION_ACTIVATE): 
            self.get_logger().info(COLOR_MAGENTA + 'Angles not able to activate' + RESET)
        if not self.change_cartesian_state(ChangeState.Request().transition.TRANSITION_ACTIVATE):
            self.get_logger().info(COLOR_MAGENTA + 'Cartesian not able to activate' + RESET)

        self.get_logger().info(COLOR_GREEN + 'Nodes activated! Ready to gait.' + RESET)

        self.get_logger().debug('Gait Planning node manager has been initialized')

    def publish_mode_to_gait_planning(self, msg):
        mode_msg = ExoMode()
        mode_msg.mode = msg.mode
        mode_msg.node_type = msg.node_type
        self.gaitplanning_mode_publisher.publish(mode_msg)

    def mode_callback(self, msg):   
        # self.get_logger().info(f'Received new mode! {msg.mode}\n')

        if msg.mode == 1:
            if self.angles_active and self.cartesian_active:
                self.change_cartesian_state(ChangeState.Request().transition.TRANSITION_DEACTIVATE)
                self.angles_active = True
                self.cartesian_active = False
                self.publish_mode_to_gait_planning(msg)
            elif self.angles_active and not self.cartesian_active:
                self.get_logger().info('Letting the gait finish')
                self.publish_mode_to_gait_planning(msg)
            elif not self.angles_active and self.cartesian_active:
                self.get_logger().info('Letting the gait finish')
                self.publish_mode_to_gait_planning(msg)
            else:
                self.get_logger().error('Both gait planning nodes inactive!')

        elif msg.node_type == 'joint_angles' and msg.mode != 1:
            if not self.angles_active:
                self.change_angles_state(ChangeState.Request().transition.TRANSITION_ACTIVATE)
            if self.cartesian_active:
                self.change_cartesian_state(ChangeState.Request().transition.TRANSITION_DEACTIVATE)
            self.angles_active = True
            self.cartesian_active = False
            self.publish_mode_to_gait_planning(msg)

        elif msg.node_type == 'cartesian' and msg.mode != 1:
            if not self.cartesian_active:
                self.change_cartesian_state(ChangeState.Request().transition.TRANSITION_ACTIVATE)
            if self.angles_active:
                self.change_angles_state(ChangeState.Request().transition.TRANSITION_DEACTIVATE)
            self.angles_active = False
            self.cartesian_active = True
            self.publish_mode_to_gait_planning(msg)

        else:
            self.get_logger().warn(f'Unknown node type: {msg.node_type}')

    def change_angles_state(self, transition: str) -> bool: 
        while not self.angles_client_change_state.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        change_angles_state_request = ChangeState.Request()
        change_angles_state_request.transition = Transition(id=transition)
        change_angles_state_future = self.angles_client_change_state.call_async(change_angles_state_request)

        self.get_logger().debug('Service has been called.')
        return self.future

    def change_cartesian_state(self, transition: str) -> bool: 
        while not self.cartesian_client_change_state.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        change_cartesian_state_request = ChangeState.Request()
        change_cartesian_state_request.transition = Transition(id=transition)
        change_cartesian_state_future = self.cartesian_client_change_state.call_async(change_cartesian_state_request)

        self.get_logger().debug('Service has been called.')
        return self.future

def main(args=None):
    rclpy.init(args=args)
    node = ServiceClient()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info('Gait Planning Manager Node started. Shut down the node using Ctrl-C.')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT) detected. Shutting down...')
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()


