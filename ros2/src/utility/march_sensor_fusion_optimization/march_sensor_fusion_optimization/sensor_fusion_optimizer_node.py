"""
Project MARCH IX, 2023-2024
Authors: Alexander James Becoy @alexanderjamesbecoy
         Alexander Andonov 
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from lifecycle_msgs.msg import State, Transition, TransitionDescription
from lifecycle_msgs.srv import GetState, ChangeState, GetAvailableTransitions
from march_shared_msgs.msg import StateEstimation, ExoMode
from std_msgs.msg import Bool, Empty
from march_sensor_fusion_optimization.parameters_handler import ParametersHandler

import time
import random as rd
import numpy as np
import tensorflow_probability as tfp
from tensorflow_probability import distributions as tfd
from tensorflow_probability import math as tfm
from bayes_opt import BayesianOptimization, UtilityFunction


class SensorFusionOptimizerNode(Node):

    def __init__(self) -> None:
        super().__init__('sensor_fusion_optimizer')

        self.declare_parameter('parameters_filepath', get_package_share_directory('march_state_estimator') + '/config/sensor_fusion_noise_parameters.yaml')

        # Bayesian Optimization parameters Optimizer
        self.max_iterations = 3
        self.dof = 3
        self.param_file = self.get_parameter('parameters_filepath').value
        self.parameter_handler = ParametersHandler(self.param_file)
        self.num_optimization_parameters = 3
        self.initial_population_size = 2
        self.performance_costs = []
        self.dt = []
        self.average_costs = []
        self.total_time = 0.0
        self.num_monte_carlo_runs = 2
        self.num_run_time_steps = 2
        self.xi = 0.0 # exploration parameter in EI function
        self.kernel_alpha = 1e-8
        self.num_optimization_parameters = 3
        self.simulation_time = 5
        # NOTE: We simplify the noise observation parameters of foot position, foot slippage,
               # and joint position to reduce dimensions of the optimization problem.
        self.bounds = np.array([
            (1e-3, 1.0),  # foot position noise (assumption: x=y=z)
            (1e-3, 1.0),  # foot slippage noise (assumption: roll=pitch=yaw)
            (1e-3, 1.0)]) # joint position noise (assumption: all joints have same position noise)

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

        # Initialize subscribers and clients
        self.state_estimation_subscriber = self.create_subscription(StateEstimation, 'state_estimation/state', self.state_estimation_callback, 10)
        self.change_state_client = self.create_client(ChangeState, 'state_estimator/change_state', callback_group=None)
        self.get_state_client = self.create_client(GetState, 'state_estimator/get_state', callback_group=None)
        self.get_available_transitions_client = self.create_client(GetAvailableTransitions, 'state_estimator/get_available_transitions', callback_group=None)
        self.exo_mode_publisher = self.create_publisher(ExoMode, 'current_mode', 10)
        self.simulation_command_publisher = self.create_publisher(Bool, 'mujoco_writer/reset', 10)

    def state_estimation_callback(self, msg: StateEstimation) -> None:
        self.performance_costs.append(msg.performance_cost)
        self.dt.append(msg.step_time)
        # self.get_logger().info('Performance cost: ' + str(msg.performance_cost) + ' with dt: ' + str(msg.step_time))

    def get_current_state(self) -> State:
        
        while not self.get_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        get_state_request = GetState.Request()
        get_state_future = self.get_state_client.call_async(get_state_request)
        rclpy.spin_until_future_complete(self, get_state_future)
        return get_state_future.result().current_state

    def get_available_transitions(self) -> list:
        
        while not self.get_available_transitions_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        get_available_transitions_request = GetAvailableTransitions.Request()
        get_available_transitions_future = self.get_available_transitions_client.call_async(get_available_transitions_request)
        rclpy.spin_until_future_complete(self, get_available_transitions_future)
        return get_available_transitions_future.result().available_transitions

    def change_state(self, transition: int) -> bool:
        self.get_logger().info('Transitioning SE state: ' + str(transition) + '...')
        
        while not self.change_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        change_state_request = ChangeState.Request()
        change_state_request.transition = Transition(id=transition)
        change_state_future = self.change_state_client.call_async(change_state_request)
        rclpy.spin_until_future_complete(self, change_state_future)

        self.get_logger().info('Service for SE transition has been called with transition: ' + str(transition) + '.')
        return change_state_future.result().success
    
    def run_filter_with_noise_parameters(self, noise_parameters: np.ndarray) -> None:
        """ Run the EKF with the provided noise parameters and collect performance costs
        through Monte Carlo simulations. """
        self.get_logger().info('Running filter with noise parameters: ' + str(noise_parameters) + '...')
        self.costs = np.zeros((self.num_monte_carlo_runs, self.num_run_time_steps))
        self.total_time = []

        for i in range(self.num_monte_carlo_runs):  # Simulate N many runs
            self.total_time.append(0.0)
            self.parameter_handler.set_optimization_parameters(noise_parameters)
            self.change_state(1)
            self.change_state(3)
            self.simulation_command_publisher.publish(Bool(data=True))
            self.exo_mode_publisher.publish(ExoMode(mode=ExoMode.STAND))
            # NOTE: Wait to be stable
            time.sleep(7)     
            self.exo_mode_publisher.publish(ExoMode(mode=ExoMode.SMALLWALK))
            for j in range(self.num_run_time_steps):
                # TODO: Make sure that the callback data runs in parallel with this 
                performance = self.performance_costs[-1]
                self.costs[i,j] = performance
                # TODO: Make sure this gets correct dt (Currently = 0.025)
                self.total_time[-1] += self.dt[-1]
                self.get_logger().info('Performance cost: ' + str(performance) + ' with dt: ' + str(self.dt[-1]))
                time.sleep(7)
            self.change_state(4)

    def evaluate_performance_cost(self) -> float:
        """Compute the JNIS performance cost of the state estimator with the current set of noise parameters."""
        average_costs_per_run = np.mean(self.costs, axis=1)
        total_time = np.sum(self.total_time)
        average_costs_per_time = average_costs_per_run / total_time
        JNIS = np.abs(np.log(np.sum(average_costs_per_time) / self.num_optimization_parameters))
        self.get_logger().info('Current JNIS: ' + str(JNIS))
        return JNIS
    
    def black_box_filter_function(self, foot_position: float, foot_slippage: float, joint_position: float) -> float:
        """ Black box function to estimate the costs of the state estimation filter. """
        # TODO: Configure parameters of State Estimation filter
        self.parameter_handler.set_optimization_parameters([
            foot_position * np.ones((3,)), 
            foot_slippage * np.ones((3,)), 
            joint_position * np.ones((3,)),
        ])
        self.run_filter_with_noise_parameters([foot_position, foot_slippage, joint_position])
        # Negative JNIS to maximize the performance
        return -self.evaluate_performance_cost()

    def optimize(self) -> np.ndarray:
        """ Optimize the EKF parameters using Bayesian Optimization with Gaussian Processes."""
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
        if not node.change_state(3):
            node.get_logger().error("Failed to transition to ACTIVATE state.")
            return
        node.exo_mode_publisher.publish(ExoMode(mode=ExoMode.STAND))
        time.sleep(5)
        node.exo_mode_publisher.publish(ExoMode(mode=ExoMode.SMALLWALK))
        time.sleep(10)
        optimized_parameters = node.optimize()
        node.get_logger().info(f"Optimization complete. Optimized parameters: {optimized_parameters}")
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT) detected. Shutting down...')
    node.destroy_node()
    rclpy.shutdown()

if "__name__" == "__main__":
    main()