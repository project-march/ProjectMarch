"""
Project MARCH IX, 2023-2024
Author: Alexander James Becoy @alexanderjamesbecoy
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from lifecycle_msgs.msg import State, Transition, TransitionDescription
from lifecycle_msgs.srv import GetState, ChangeState, GetAvailableTransitions
from march_shared_msgs.msg import StateEstimation

from march_sensor_fusion_optimization.bayesian_optimization import BayesianOptimizer
from march_sensor_fusion_optimization.state_handler import StateHandler, BayesianOptimizationStates, BayesianOptimizationTransitions

class SensorFusionOptimizerNode(Node):

    def __init__(self) -> None:
        super().__init__('sensor_fusion_optimizer')

        self.declare_parameter('max_iterations', 1000)
        self.declare_parameter('min_observation_change', 1e-6)
        self.declare_parameter('parameters_filepath', get_package_share_directory('march_state_estimator') + '/config/sensor_fusion_noise_parameters.yaml')

        # Initialize Bayesian Optimizer
        self.max_iterations = self.get_parameter('max_iterations').value
        self.min_observation_change = self.get_parameter('min_observation_change').value
        self.bayesian_optimizer = BayesianOptimizer(
            dof=3.0,
            max_iter=self.max_iterations, 
            min_obs=self.min_observation_change, 
            param_file=self.get_parameter('parameters_filepath').value
        )

        # Initialize state machine
        self.state_machine = StateHandler(initial_state=BayesianOptimizationStates.STATE_CONFIGURATION)
        self.timer = self.create_timer(self.state_machine.timer_period, self.timer_callback)

        # Initialize subscribers and clients
        self.state_estimation_subscriber = self.create_subscription(StateEstimation, 'state_estimation/state', self.state_estimation_callback, 10)
        self.change_state_client = self.create_client(ChangeState, 'state_estimator/change_state', callback_group=None)
        self.get_state_client = self.create_client(GetState, 'state_estimator/get_state', callback_group=None)
        self.get_available_transitions_client = self.create_client(GetAvailableTransitions, 'state_estimator/get_available_transitions', callback_group=None)

        self.get_logger().info('Sensor Fusion Optimizer Node has been initialized.')


    def timer_callback(self) -> None:
        current_state = self.state_machine.get_current_state()

        if current_state == BayesianOptimizationStates.STATE_CONFIGURATION:
            self.bayesian_optimizer.configure(population_size=10)
            self.state_machine.transition_state(BayesianOptimizationTransitions.TRANSITION_CONFIGURATION_INITIALIZATION)


    def state_estimation_callback(self, msg: StateEstimation) -> None:
        self.bayesian_optimizer.performance_costs.append(msg.performance_cost)


    def get_current_state(self) -> State:
        self.get_logger().info('Getting current state...')
        
        while not self.get_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        get_state_request = GetState.Request()
        get_state_future = self.get_state_client.call_async(get_state_request)
        rclpy.spin_until_future_complete(self, get_state_future)

        self.get_logger().info('Service has been called.')
        return get_state_future.result().current_state


    def get_available_transitions(self) -> list:
        self.get_logger().info('Getting available transitions...')
        
        while not self.get_available_transitions_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        get_available_transitions_request = GetAvailableTransitions.Request()
        get_available_transitions_future = self.get_available_transitions_client.call_async(get_available_transitions_request)
        rclpy.spin_until_future_complete(self, get_available_transitions_future)

        self.get_logger().info('Service has been called.')
        return get_available_transitions_future.result().available_transitions


    def change_state(self, transition: str) -> bool:
        self.get_logger().info('Transitioning via ' + transition + '...')
        
        while not self.change_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        change_state_request = ChangeState.Request()
        change_state_request.transition = Transition(id=transition)
        change_state_future = self.change_state_client.call_async(change_state_request)
        rclpy.spin_until_future_complete(self, change_state_future)

        self.get_logger().info('Service has been called.')
        return change_state_future.result().success
            


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionOptimizerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info('Sensor Fusion Optimizer Node started. Shut down the node using Ctrl-C.')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT) detected. Shutting down...')
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
