"""
Project MARCH IX, 2023-2024
Authors: Alexander James Becoy @alexanderjamesbecoy
         Alexander Andonov 
"""

import time   
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from lifecycle_msgs.srv import ChangeState
from march_shared_msgs.msg import StateEstimation, ExoMode
from std_msgs.msg import Bool, Float64MultiArray
from march_sensor_fusion_optimization.parameters_handler import ParametersHandler
from bayes_opt import BayesianOptimization, UtilityFunction


class SensorFusionOptimizerNode(Node):

    def __init__(self) -> None:
        super().__init__('sensor_fusion_optimizer')

        self.declare_parameter('parameters_filepath', get_package_share_directory('march_state_estimator') + '/config/sensor_fusion_noise_parameters.yaml')
        self.current_dir = os.path.dirname(os.path.realpath(__file__))

        # Bayesian Optimization parameters
        self.max_iterations = 100
        self.param_file = self.get_parameter('parameters_filepath').value
        self.parameter_handler = ParametersHandler(self.param_file)
        self.num_optimization_parameters = 3
        self.initial_population_size = 30
        self.performance_costs = []
        self.dt = 0.025 # NOTE: Check unit and value from SE
        self.total_time = 0.0
        self.num_monte_carlo_runs = 200
        self.xi = 0.0  # exploration parameter in EI function
        self.kernel_alpha = 1e-8
        self.simulation_time = 12 # [seconds]
        # NOTE: Simplified noise observation parameters to reduce optimization problem dimensions
        self.bounds = np.array([
            (1e-3, 1.),  # foot position noise (assumption: x=y=z)
            (1e-3, 1.),  # foot slippage noise (assumption: roll=pitch=yaw)
            (1e-3, 1.)]) # joint position noise (assumption: all joints have same position noise)

        self.utility_function = UtilityFunction(kind="ei", kappa=2.5, xi=self.xi)
        self.optimizer = BayesianOptimization(
            f=self.black_box_filter_function,  
            pbounds={"foot_position": self.bounds[0], "foot_slippage": self.bounds[1], "joint_position": self.bounds[2]},
            verbose=2,
            random_state=1
        )
        # TODO: Implement Student-t process for surrogate model
        # Kernel parameters
        self.optimizer.set_gp_params(
            alpha=self.kernel_alpha,
            normalize_y=True,
            n_restarts_optimizer=5
        )

        # Initialize subscribers, publishers, and clients
        self.state_estimation_subscriber = self.create_subscription(StateEstimation, 'state_estimation/state', self.state_estimation_callback, 10)
        self.change_state_client = self.create_client(ChangeState, 'state_estimator/change_state')
        self.exo_mode_publisher = self.create_publisher(ExoMode, 'current_mode', 10)
        self.parameters_publisher = self.create_publisher(Float64MultiArray, 'state_estimation/noise_parameters', 10)
        self.simulation_command_publisher = self.create_publisher(Bool, 'mujoco/writer/reset', 10)

        self.collect_performance = False
        self.sensor_fusion_valid = True

    def state_estimation_callback(self, msg: StateEstimation) -> None:
        if self.collect_performance:
            self.performance_costs.append(msg.performance_cost)
            self.total_time += msg.step_time
            self.sensor_fusion_valid = msg.sensor_fusion_valid
            # self.get_logger().info(f'Latest performance cost: {self.performance_costs[-1]}')
        else: 
            # self.get_logger().info('Not in test mode, skipping performance cost collection...')
            self.total_time += msg.step_time
            self.sensor_fusion_valid = msg.sensor_fusion_valid

    def change_state(self, transition: int) -> bool:
        # self.get_logger().info(f'Transitioning SE state: {transition}')
        
        while not self.change_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        change_state_request = ChangeState.Request()
        change_state_request.transition.id = transition
        change_state_future = self.change_state_client.call_async(change_state_request)
        rclpy.spin_until_future_complete(self, change_state_future)

        return change_state_future.result().success

    # TODO: Find an elegant way to collect data through parallel threads
    def collect_messages_for_duration(self, duration):
        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        while self.get_clock().now().seconds_nanoseconds()[0] - start_time < duration:
            rclpy.spin_once(self, timeout_sec=0.1)

    def run_filter_with_noise_parameters(self, noise_parameters: np.ndarray) -> None:
        """Run the EKF with the provided noise parameters and collect performance costs through Monte Carlo simulations."""
        self.get_logger().info(f'Running filter with noise parameters: {noise_parameters}')
        self.costs = [[] for _ in range(self.num_monte_carlo_runs)]
        self.total_time = 0.0

        for i in range(self.num_monte_carlo_runs):

            # self.get_logger().info(f'Run {i+1}/{self.num_monte_carlo_runs}')
            self.change_state(1) # Configure
            self.parameters_publisher.publish(Float64MultiArray(data=noise_parameters))
            time.sleep(1)
            self.change_state(3) # Activate
            if self.sensor_fusion_valid:
                self.simulation_command_publisher.publish(Bool(data=True)) # Restart simulation
                time.sleep(2)
                self.exo_mode_publisher.publish(ExoMode(mode=ExoMode.STAND, node_type='joint_angles'))
                self.collect_performance = True
                self.collect_messages_for_duration(self.simulation_time/3)
                self.exo_mode_publisher.publish(ExoMode(mode=ExoMode.SMALLWALK, node_type='cartesian'))
                self.collect_messages_for_duration(self.simulation_time)
                self.exo_mode_publisher.publish(ExoMode(mode=ExoMode.STAND, node_type='joint_angles'))
                self.collect_messages_for_duration(self.simulation_time/3) # Wait for stability, otherwise cleanup crashes
                self.costs[i] = self.performance_costs
                self.performance_costs = []
                self.collect_performance = False
            else:
                self.get_logger().error('Sensor fusion is not valid. Skipping Monte Carlo runs for the current params...')
                self.costs[i] = [float('inf')]
                self.performance_costs = []
                self.collect_performance = False
                self.change_state(4) # Deactivate
                self.change_state(2) # Cleanup
                break
            self.change_state(4) # Deactivate
            self.change_state(2) # Cleanup

    def evaluate_performance_cost(self) -> float:
        """Compute the JNIS performance cost of the state estimator with the current set of noise parameters."""
        if not self.costs:
            return float('inf')
        
        # Some runs receive more messages than others
        min_length = min(len(arr) for arr in self.costs)
        trimmed_costs = [arr[:min_length] for arr in self.costs]
        trimmed_costs_array = np.array(trimmed_costs)
        average_costs_per_run = np.mean(trimmed_costs_array)
        average_costs_per_time = average_costs_per_run / self.total_time
        JNIS = np.abs(np.log(average_costs_per_time / self.num_optimization_parameters))
        self.get_logger().info(f'Current JNIS: {JNIS}')
        return JNIS

    def black_box_filter_function(self, foot_position: float, foot_slippage: float, joint_position: float) -> float:
        """Black box function to estimate the costs of the state estimation filter."""
        self.parameter_handler.set_optimization_parameters([
            foot_position * np.ones((3,)), 
            foot_slippage * np.ones((3,)), 
            joint_position * np.ones((3,))
        ])
        self.run_filter_with_noise_parameters([foot_position, foot_slippage, joint_position])
        # Return the negative JNIS to maximize the performance
        jnis = self.evaluate_performance_cost()
        # Record in file for each run
        with open(self.current_dir + '/last_run_record.txt', 'a') as f:
            f.write(f"Foot_position: {foot_position} | Foot_slippage: {foot_slippage} | Joint_position: {joint_position} | JNIS: {jnis}\n")
        return -jnis

    def optimize(self) -> np.ndarray:
        """Optimize the EKF parameters using Bayesian Optimization with Gaussian Processes."""
        self.optimizer.maximize(
            acquisition_function=self.utility_function,
            init_points=self.initial_population_size,
            n_iter=self.max_iterations
        )
        print("Minimum found JNIS: ", -(self.optimizer.max["target"]))
        return self.optimizer.max["params"]

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionOptimizerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info('Sensor Fusion Optimizer Node started.')
        if not node.change_state(1):
            node.get_logger().error("Failed to transition to CONFIGURE state.")
            return
        node.simulation_command_publisher.publish(Bool(data=True))
        # Wait for stability
        time.sleep(3)
        if not node.change_state(3):
            node.get_logger().error("Failed to transition to ACTIVATE state.")
            return
        node.exo_mode_publisher.publish(ExoMode(mode=ExoMode.STAND, node_type='joint_angles'))
        time.sleep(5)
        node.exo_mode_publisher.publish(ExoMode(mode=ExoMode.SMALLWALK, node_type='cartesian'))
        time.sleep(10)
        optimized_parameters = node.optimize()
        node.get_logger().info(f"Optimization complete. Optimized parameters: {optimized_parameters}")
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT) detected. Shutting down...')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
