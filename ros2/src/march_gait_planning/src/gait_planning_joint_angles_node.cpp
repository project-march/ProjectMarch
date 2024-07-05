/*
Authors: Femke Buiks, MIX

This file contains the functionality of the GaitPlanningAnglesNode. This node serves to supply gaits to the exo that are predetermined joint angles (from csv files).
Gait logic is mainly located here, as the publishing of gaits is dependent on callbacks from the state machine. 

*/ 

#include "march_gait_planning/gait_planning_joint_angles_node.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using std::placeholders::_1; 

int INTERPOLATING_TIMESTEPS = 400;

#define COLOR_GREEN   "\033[32m"
#define RESET   "\033[0m"
#define COLOR_PERIWINKLE   "\033[38;5;147m"

struct CSVRow {
    std::string left_hip_aa;
    std::string left_hip_fe;
    std::string left_knee; 
    std::string left_ankle; 
    std::string right_hip_aa;
    std::string right_hip_fe;
    std::string right_knee; 
    std::string right_ankle;
};

GaitPlanningAnglesNode::GaitPlanningAnglesNode()
 : rclcpp_lifecycle::LifecycleNode("gait_planning_angles_node", rclcpp::NodeOptions().use_intra_process_comms(false))
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GaitPlanningAnglesNode::on_configure(const rclcpp_lifecycle::State &state) {

    (void) state; 
    m_gait_planning = GaitPlanningAngles(); 
    m_first_stand = true; 
    m_active = false; 
    m_single_execution_done = false; 

    // m_joints_msg.joint_names = {"left_hip_aa", "left_hip_fe", "left_knee", "left_ankle", 
    //                     "right_hip_aa", "right_hip_fe", "right_knee", "right_ankle"};

    m_current_state_subscriber = this->create_subscription<march_shared_msgs::msg::StateEstimation>("state_estimation/state", 10, std::bind(&GaitPlanningAnglesNode::currentJointAnglesCallback, this, _1));
    m_exo_mode_subscriber = this->create_subscription<march_shared_msgs::msg::ExoMode>("gait_planning_mode", 10, std::bind(&GaitPlanningAnglesNode::currentModeCallback, this, _1));
    m_joint_angle_trajectory_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("march_joint_position_controller/commands", 10);

    m_gait_planning.setGaitType(ExoMode::BootUp);
    m_gait_planning.setPrevGaitType(ExoMode::BootUp); 
    m_gait_planning.setStanceFoot(DOUBLE_STANCE_LEG); 
    // change sethomestand to parsing yaml aswell? Or still use gaitfiles as these should be the same? 

    std::string homestand_path = ament_index_cpp::get_package_share_directory("march_gait_planning") + "/m9_gait_files/homestand.yaml";
    m_gait_planning.setHomeStand(parseHomestandYAML(homestand_path)); 

    if (m_gait_planning.getHomeStand().size() != 8) {
        RCLCPP_WARN(this->get_logger(), "Unexpected number of values in homestand, %d", m_gait_planning.getHomeStand().size()); 
    } else {
        RCLCPP_INFO(this->get_logger(), "Successful retrieval of homestand: " COLOR_PERIWINKLE "%f, %f, %f, %f, %f, %f, %f, %f" RESET, m_gait_planning.getHomeStand()[0], m_gait_planning.getHomeStand()[1], m_gait_planning.getHomeStand()[2], m_gait_planning.getHomeStand()[3], m_gait_planning.getHomeStand()[4], m_gait_planning.getHomeStand()[5], m_gait_planning.getHomeStand()[6], m_gait_planning.getHomeStand()[7]); 
    }

    RCLCPP_DEBUG(this->get_logger(), COLOR_GREEN "Joint angles node configured!" RESET);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GaitPlanningAnglesNode::on_activate(const rclcpp_lifecycle::State &state) {

    (void) state;
    m_joint_angle_trajectory_publisher->on_activate(); 

    m_active = true;  

    RCLCPP_DEBUG(this->get_logger(), "Joint angles node activated!"); 

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GaitPlanningAnglesNode::on_deactivate(const rclcpp_lifecycle::State &state) {

    (void) state; 
    m_joint_angle_trajectory_publisher->on_deactivate(); 
    RCLCPP_DEBUG(this->get_logger(), "Joint angles node deactivated!"); 

    m_active = false; 

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GaitPlanningAnglesNode::on_cleanup(const rclcpp_lifecycle::State &state) {
    
    (void) state; 
    m_joint_angle_trajectory_publisher.reset();  
    RCLCPP_DEBUG(this->get_logger(), "Joint angles node cleaned up!"); 

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GaitPlanningAnglesNode::on_shutdown(const rclcpp_lifecycle::State &state) {

    (void) state; 
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void GaitPlanningAnglesNode::setFirstCallbackMsg(const march_shared_msgs::msg::ExoMode::SharedPtr msg){
    m_first_callback_msg = msg; 
}

void GaitPlanningAnglesNode::currentModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "Received current mode: " COLOR_PERIWINKLE "%s " RESET, toString(static_cast<ExoMode>(msg->mode)).c_str()); 
    if (m_active){
        // RCLCPP_INFO(this->get_logger(), "m_active = true"); 
        m_gait_planning.setPrevGaitType(m_gait_planning.getGaitType());
        m_gait_planning.setGaitType((ExoMode)msg->mode);
        // DO NOT set counter to 0 if you switch from walking to standing (prev type is walk and new type is stand) 
        if ((ExoMode)msg->mode != ExoMode::Stand){
            m_gait_planning.setCounter(0); 
            RCLCPP_DEBUG(this->get_logger(), "setting counter to 0 in this gait switch!");
        }

    if ((ExoMode)msg->mode == ExoMode::Sit){
        m_single_execution_done = false; 
    }

    if ((ExoMode)msg->mode == ExoMode::Stand && m_gait_planning.getPrevGaitType() == ExoMode::Sit){
        // m_gait_planning.setCounter(0); 
        m_single_execution_done = false; 
    }

        if (!m_first_stand){
            publishJointTrajectoryPoints(); 
        }

    } else {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "not active"); 
    }

}

void GaitPlanningAnglesNode::currentJointAnglesCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg) {
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "\n entered joint angles callback \n"); 
    if (m_active){
        if (m_first_stand && (m_gait_planning.getGaitType() == ExoMode::Stand || m_gait_planning.getGaitType() == ExoMode::Sit)) {
        std::vector<double> point = msg->joint_state.position; 
        if (point.size() >= 8) {
            m_gait_planning.setPrevPoint({point[0], point[2], point[3], point[4], point[5], point[7], point[8], point[9]}); 
            RCLCPP_INFO(this->get_logger(), "Received current joint angles"); 
            std::string point_str;
            for (const auto &value : m_gait_planning.getPrevPoint()) {
                point_str += std::to_string(value) + " ";
            }

            RCLCPP_DEBUG(this->get_logger(), "Point values: %s", point_str.c_str());
            m_first_stand = false;
        } else {
            RCLCPP_INFO(this->get_logger(), "Not enough joint angles to set previous point correctly!");
        }
    }
    if (m_gait_planning.getCounter() >= (int)(m_current_trajectory.size()-1)){
    
        // RCLCPP_INFO(this->get_logger(), "New stance foot: %d", m_gait_planning.getStanceFoot());
        
        switch (m_gait_planning.getGaitType()){
            case ExoMode::Walk:
            case ExoMode::SidewaysRight:
                m_gait_planning.setCounter(0); 
                break;

            default: 
                break;

        }
    }
    publishJointTrajectoryPoints(); 
    } else {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "not active"); 
    }
    
}

void GaitPlanningAnglesNode::processMovingGaits(const int &counter){
    // TODO: Whenever this function is called, we first need to check whether the trajectory is empty BEFORE IT IS FILLED WITH THE NEW GAIT, and empty it before filling it with the new gait.
    if (!m_current_trajectory.empty()){
        m_joints_msg.data = m_current_trajectory[counter];
        m_joint_angle_trajectory_publisher->publish(m_joints_msg);
        RCLCPP_DEBUG(this->get_logger(), "Gait message published!"); 
    }
} 

std::vector<double> GaitPlanningAnglesNode::remapKinematicChaintoAlphabetical(const std::vector<double> &kinematic_vector){
    return {kinematic_vector[3], kinematic_vector[0], kinematic_vector[1], kinematic_vector[2],
            kinematic_vector[7], kinematic_vector[4], kinematic_vector[5], kinematic_vector[6]};
}

void GaitPlanningAnglesNode::processHomeStandGait(){
    if (m_gait_planning.getCounter() == 0){ // When switching to homestand
        m_incremental_steps_to_home_stand.clear();
        for (unsigned i = 0; i < m_gait_planning.getHomeStand().size(); ++i) {
                m_incremental_steps_to_home_stand.push_back((m_gait_planning.getHomeStand()[i] - m_gait_planning.getPrevPoint()[i]) / INTERPOLATING_TIMESTEPS); // 40 iterations to reach the target, i.e. in 2 seconds
        }
        m_initial_point = m_gait_planning.getPrevPoint();
        RCLCPP_DEBUG(this->get_logger(), "Increments correctly calculated!");
    }
    std::vector<double> temp_moving_to_home_stand;
   
    if (m_gait_planning.getCounter() < INTERPOLATING_TIMESTEPS){
        RCLCPP_DEBUG(this->get_logger(), "Moving towards home stand!");
        for (unsigned i = 0; i < m_gait_planning.getHomeStand().size(); ++i) {
            m_initial_point[i] += m_incremental_steps_to_home_stand[i];
            temp_moving_to_home_stand.push_back(m_initial_point[i]);
        } 

    }
    else{
        RCLCPP_DEBUG(this->get_logger(), "Home stand position reached!");
        temp_moving_to_home_stand = m_gait_planning.getHomeStand();
        m_initial_point.clear();
    }

    m_joints_msg.data = temp_moving_to_home_stand;
    m_joint_angle_trajectory_publisher->publish(m_joints_msg);
    
    m_gait_planning.setCounter(m_gait_planning.getCounter() + 1);
}

void GaitPlanningAnglesNode::processBootUpToStandGait(){
    if (m_gait_planning.getCounter() == 0){ 
        // This is the moment we go from BootUp to Stand, so we obtain the current joint angles and calculate the increments to reach the sit position
        m_incremental_steps_to_home_stand.clear();
        for (unsigned i = 0; i < m_gait_planning.getHomeStand().size(); ++i) {
                m_incremental_steps_to_home_stand.push_back((m_gait_planning.getSitToStandGait()[0][i] - m_gait_planning.getPrevPoint()[i]) / INTERPOLATING_TIMESTEPS); // 40 iterations to reach the target, i.e. in 2 seconds
        }
        m_initial_point = m_gait_planning.getPrevPoint();
        RCLCPP_DEBUG(this->get_logger(), "Increments correctly calculated!");
    }
    std::vector<double> temp_moving_to_home_stand;
   
    if (m_gait_planning.getCounter() < INTERPOLATING_TIMESTEPS){
        RCLCPP_DEBUG(this->get_logger(), "Moving towards sit position!");
        for (unsigned i = 0; i < m_gait_planning.getHomeStand().size(); ++i) {
            m_initial_point[i] += m_incremental_steps_to_home_stand[i];
            temp_moving_to_home_stand.push_back(m_initial_point[i]);
        }
    }
    else{
        RCLCPP_DEBUG(this->get_logger(), "Sit Position reached");
        temp_moving_to_home_stand = m_gait_planning.getSitToStandGait()[0];
        m_initial_point.clear();
    }

    m_joints_msg.data = temp_moving_to_home_stand;
    m_joint_angle_trajectory_publisher->publish(m_joints_msg);
    
    m_gait_planning.setCounter(m_gait_planning.getCounter() + 1);
}

void GaitPlanningAnglesNode::finishGaitBeforeStand(){
    unsigned count = m_gait_planning.getCounter(); 
    if (count < m_current_trajectory.size()-1){ 
        processMovingGaits(count); 
        m_gait_planning.setCounter(count+1); 
        // RCLCPP_INFO(this->get_logger(), "Finishing gait, with count: %d", count);
    } if (count == m_current_trajectory.size()-1) { 
        m_joints_msg.data = m_gait_planning.getHomeStand(); 
        m_joint_angle_trajectory_publisher->publish(m_joints_msg);
        m_gait_planning.setPrevPoint(m_gait_planning.getHomeStand());
        m_gait_planning.setStanceFoot(DOUBLE_STANCE_LEG); 
        RCLCPP_DEBUG(this->get_logger(), "Publishing home stand");
    } 
}


void GaitPlanningAnglesNode::publishJointTrajectoryPoints(){
    if (m_first_stand){
        return; 
    }

    unsigned count = m_gait_planning.getCounter(); 
    ExoMode current_gait = m_gait_planning.getGaitType(); 
    ExoMode previous_gait = m_gait_planning.getPrevGaitType(); 


        switch (current_gait) {
            case ExoMode::Walk : 
            
            switch(previous_gait){
                case ExoMode::Walk : 
                    RCLCPP_DEBUG(this->get_logger(), "getting half step with stance foot %d", m_gait_planning.getStanceFoot()); 
                    m_current_trajectory = m_gait_planning.getFullGaitAngleCSV(); 
                    processMovingGaits(count);
                    if (count >= (m_current_trajectory.size()-2)){
                        RCLCPP_INFO(this->get_logger(), "setting a new stance foot"); 
                        m_gait_planning.setStanceFoot((m_gait_planning.getStanceFoot() == LEFT_STANCE_LEG) ? RIGHT_STANCE_LEG : LEFT_STANCE_LEG);
                    }
                    m_gait_planning.setCounter((count >= (m_current_trajectory.size() - 1)) ? 0 : (count + 1));
                    break; 

                case ExoMode::Stand :

                    RCLCPP_DEBUG(this->get_logger(), "getting first step with stance foot %d", m_gait_planning.getStanceFoot()); 
                    m_current_trajectory = m_gait_planning.getFirstStepAngleTrajectory(); 
                    RCLCPP_DEBUG(this->get_logger(), "count: %d", count); 
                    processMovingGaits(count);
                    m_gait_planning.setCounter((count >= (m_current_trajectory.size() - 1)) ? 0 : (count + 1));
                    if (m_gait_planning.getCounter() >= (int)(m_current_trajectory.size()-1)){
                        RCLCPP_DEBUG(this->get_logger(), "setting previous gait type to walk"); 
                        m_gait_planning.setStanceFoot(LEFT_STANCE_LEG); 
                        m_gait_planning.setPrevGaitType(ExoMode::Walk); 
                    } 
                    break; 
                default :
                    break; 
                }
                break;
            
            case ExoMode::Stand :

                switch(previous_gait){

                    case ExoMode::Sit :
                        // if counter= size trajectory refill with sit to stand, otherwise finish gait and reset to 0 afterwards 
                        if (count+1 < m_current_trajectory.size() && !m_single_execution_done){
                            processMovingGaits(count);
                            m_gait_planning.setCounter(count+1); 
                        } else if (count+1 >= m_current_trajectory.size() && !m_single_execution_done){
                            m_gait_planning.setCounter(0); 
                            m_single_execution_done = true; 
                            m_current_trajectory = m_gait_planning.getSitToStandGait(); 
                        } else {
                            processMovingGaits(count);
                            // // RCLCPP_INFO(this->get_logger(), "count: %d", count); 
                            m_gait_planning.setCounter((count >= (m_current_trajectory.size() - 1)) ? (m_current_trajectory.size() - 1) : (count + 1));
                            if (m_gait_planning.getCounter() >= (int)(m_current_trajectory.size()-1)){
                                m_gait_planning.setPrevGaitType(ExoMode::Stand); 
                            }
                        }
                        break; 

                    case ExoMode::Hinge :
                        
                        m_current_trajectory = m_gait_planning.getHingeGait(); 
                        processMovingGaits(count); 
                        // RCLCPP_INFO(this->get_logger(), "Count: %d", count);
                        m_gait_planning.setCounter((count <= 0) ? 0 : (count - 1));
                        if (m_gait_planning.getCounter() <= 0){
                            m_current_trajectory.clear();
                            m_gait_planning.setPrevGaitType(ExoMode::Stand); 
                        } 
                        break;

                    case ExoMode::Walk :
                        // RCLCPP_INFO(this->get_logger(), "count: %d", count); 
                        if (count < m_current_trajectory.size()-1){
                            RCLCPP_DEBUG(this->get_logger(), "finishing gait! with count %d", count); 
                            processMovingGaits(count); 
                            m_gait_planning.setCounter(count+1); 
                        } if (count >= m_current_trajectory.size()-1){
                            // run through step close trajectory here instead of in finishGaitBeforeStand
                            m_gait_planning.setCounter(0); 
                            RCLCPP_DEBUG(this->get_logger(), "getting step close with stance foot %d", m_gait_planning.getStanceFoot()); 
                            m_current_trajectory = m_gait_planning.getStepCloseGait();
                            RCLCPP_DEBUG(this->get_logger(), "Filled step close trajectory, size = %d", m_current_trajectory.size()); 
                            m_gait_planning.setStanceFoot(DOUBLE_STANCE_LEG); 
                            m_gait_planning.setPrevGaitType(ExoMode::Stand); 
                        }
                        break; 

                    case ExoMode::Stand :
                    case ExoMode::Ascending :
                    case ExoMode::Descending :
                    case ExoMode::SidewaysRight :
                    case ExoMode::SidewaysLeft :

                        finishGaitBeforeStand(); 
                        break; 

                    case ExoMode::BootUp :

                        processHomeStandGait(); 
                        break; 

                    default :

                        break; 
                }
                break; 
            
            case ExoMode::Sit :
                if (previous_gait == ExoMode::BootUp){
                    // RCLCPP_INFO(this->get_logger(), "BootUp mode");
                    processBootUpToStandGait();
                    break;
                } else {
                    m_current_trajectory = m_gait_planning.getStandToSitGait(); 
                    processMovingGaits(count);
                    m_gait_planning.setCounter((count >= (m_current_trajectory.size() - 1)) ? (m_current_trajectory.size() - 1) : (count + 1));
                    // RCLCPP_INFO(this->get_logger(), "count: %d", m_gait_planning.getCounter()); 
                    break;
                }
                

            case ExoMode::Hinge:
                m_current_trajectory = m_gait_planning.getHingeGait(); 
                processMovingGaits(count); 
                m_gait_planning.setCounter((count >= (m_current_trajectory.size()-1)) ? (m_current_trajectory.size()-1): (count + 1)); 
                // RCLCPP_INFO(this->get_logger(), "count: %d", m_gait_planning.getCounter());
                // RCLCPP_INFO(this->get_logger(), "count: %d", m_gait_planning.getCounter());
                break;
            
            case ExoMode::SidewaysRight :
                m_current_trajectory = m_gait_planning.getSidewaysRightGait(); 
                processMovingGaits(count); 
                m_gait_planning.setCounter((count >= (m_current_trajectory.size() - 1)) ? 0 : (count + 1));
                break;

            case ExoMode::SidewaysLeft:
                m_current_trajectory = m_gait_planning.getSidewaysLeftGait(); 
                processMovingGaits(count); 
                m_gait_planning.setCounter((count >= (m_current_trajectory.size() - 1)) ? 0 : (count + 1));
                break;

            case ExoMode::Ascending :
                m_current_trajectory = m_gait_planning.getAscendingGait(); 
                if (count < m_current_trajectory.size()-1){
                    processMovingGaits(count); 
                    m_gait_planning.setCounter(count+1); 
                }
                break;

            case ExoMode::Descending :
                m_current_trajectory = m_gait_planning.getDescendingGait(); 
                if (count < m_current_trajectory.size()-1){
                    processMovingGaits(count); 
                    m_gait_planning.setCounter(count+1); 
                }
                break;

            default :
                break;
        }
    }

std::vector<double> GaitPlanningAnglesNode::parseHomestandYAML(const std::string& file_path){

    std::vector<double> values;
    
    try {
        YAML::Node config = YAML::LoadFile(file_path);
        YAML::Node joint_angles = config["joint_angles"];
        if (joint_angles && joint_angles.IsSequence()) {
            for (const auto& value : joint_angles) {
                values.push_back(value.as<double>());
            }
        }
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error parsing YAML file: %s", e.what());
    }

    return values;
}

int main(int argc, char *argv[]){
    
    rclcpp::init(argc, argv); 

    rclcpp::executors::SingleThreadedExecutor executor; 
    std::shared_ptr<GaitPlanningAnglesNode> gait_planning_node = std::make_shared<GaitPlanningAnglesNode>();
    executor.add_node(gait_planning_node->get_node_base_interface()); 
    executor.spin(); 

    rclcpp::shutdown(); 

    return 0; 
}