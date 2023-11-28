#ifndef IK_SOLVER__TASK_HPP
#define IK_SOLVER__TASK_HPP

#include <algorithm>
#include <string>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

class Task
{
    public:
        Task();     // Default constructor.
        Task (      // Constructor. TODO: Load the task parameters from a YAML file
            int task_id,            // ID of the task
            std::string task_name,  // Name of the task
            int task_m,             // Dimension of the task
            int task_n              // Dimension of the joint space
        );       

        std::string getTaskName();                      // Get the name of the task
        int getTaskID();                                // Get the ID of the task
        int getTaskM();                                 // Get the dimension of the task
        int getTaskN();                                 // Get the dimension of the joint space

        Eigen::VectorXf solve();                        // Solve the task

        void setDesiredPose(Eigen::VectorXf  * desired_pose);  // Set the desired pose of the task
        void setCurrentPose(Eigen::VectorXf  * current_pose);  // Set the current pose of the task

        void setGainP(float gain_p);                    // Set the proportional gain
        // void setGainI(float gain_i);                    // Set the integral gain
        // void setGainD(float gain_d);                    // Set the derivative gain

        const Eigen::MatrixXf * getJacobian();          // Get the Jacobian
        const Eigen::MatrixXf * getJacobianInverse();   // Get the inverse of Jacobian
        void calculateJacobian();                       // Calculate the Jacobian

    private:
    
        Eigen::VectorXf calculateError();               // Calculate the error
        // Eigen::Vector3f calculateIntegralError();       // Calculate the integral error
        // Eigen::Vector3f calculateDerivativeError();     // Calculate the derivative error
        void calculateJacobianInverse();     // Calculate the inverse of Jacobian

        std::string task_name_;                 // Name of the task
        int task_id_;                           // ID of the task
        int task_m_;                            // Dimension of the task
        int task_n_;                            // Dimension of the joint space
        Eigen::VectorXf * desired_pose_;        // Desired pose of the task
        Eigen::VectorXf * current_pose_;        // Current pose of the task
        float gain_p_ = 0.0;                    // Proportional gain. Default value is 0.0
        float gain_i_ = 0.0;                    // Integral gain. Default value is 0.0
        float gain_d_ = 0.0;                    // Derivative gain. Default value is 0.0
        Eigen::MatrixXf jacobian_;              // Jacobian matrix
        Eigen::MatrixXf jacobian_inverse_;      // Inverse of Jacobian matrix
        Eigen::VectorXf previous_error_;        // Previous error
        Eigen::VectorXf integral_error_;        // Integral error
};

#endif  // IK_SOLVER__TASK_HPP