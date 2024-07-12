/*Authors: Femke Buiks and Andrew Hutani, MIX*/

#include "march_gait_planning/gait_planning_node.hpp"
#include "../../march_mode_machine/include/march_mode_machine/exo_mode.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "../logging_colors.hpp"


using std::placeholders::_1; 


GaitPlanningCartesianNode::GaitPlanningCartesianNode()
    : rclcpp_lifecycle::LifecycleNode("gait_planning_cartesian_node", rclcpp::NodeOptions().use_intra_process_comms(false))
 {
 }

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GaitPlanningCartesianNode::on_configure(const rclcpp_lifecycle::State &state){

    (void) state; 
    m_gait_planning = GaitPlanning(); 
    m_desired_footpositions_msg = std::make_shared<march_shared_msgs::msg::IksFootPositions>();
    m_pose = std::make_shared<geometry_msgs::msg::Pose>(); 
    m_visualization_msg_rviz = std::make_shared<geometry_msgs::msg::PoseArray>(); 
    m_single_execution_done = false; 
    m_variable_first_step_done = false; 
    m_active = false; 
    m_home_stand_trajectory.clear(); 

    m_iks_foot_positions_publisher = this->create_publisher<march_shared_msgs::msg::IksFootPositions>("ik_solver/buffer/input", 10);

    m_interpolated_bezier_visualization_publisher_rviz = this->create_publisher<geometry_msgs::msg::PoseArray>("bezier_visualization", 10); 

    m_exo_mode_subscriber = create_subscription<march_shared_msgs::msg::ExoMode>(
        "gait_planning_mode", 10, std::bind(&GaitPlanningCartesianNode::currentModeCallback, this, _1)); 
    m_exo_joint_state_subscriber = create_subscription<march_shared_msgs::msg::StateEstimation>(
        "state_estimation/state", 100, std::bind(&GaitPlanningCartesianNode::currentExoJointStateCallback, this, _1)); 

    // m_variable_foot_step_subscriber = create_subscription<march_shared_msgs::msg::FootStepOutput>("footsteps", 100, std::bind(&GaitPlanningCartesianNode::MPCCallback, this, _1)); 

    m_mpc_foot_positions_subscriber = create_subscription<geometry_msgs::msg::PoseArray>("mpc_solver/buffer/output", 10, std::bind(&GaitPlanningCartesianNode::MPCCallback, this, _1));

    m_gait_planning.setGaitType(ExoMode::BootUp); 
    
    std::string homestand_path = ament_index_cpp::get_package_share_directory("march_gait_planning") + "/m9_gait_files/homestand.yaml";
    m_home_stand = parseHomestandYAML(homestand_path);

    if (m_home_stand.size() != 6) {
        RCLCPP_WARN(this->get_logger(), "Unexpected number of values in homestand, %d", m_home_stand.size()); 
    } else {
        RCLCPP_INFO(this->get_logger(), "Successful retrieval of homestand: " COLOR_PERIWINKLE "%f, %f, %f, %f, %f, %f" RESET, m_home_stand[0], m_home_stand[1], m_home_stand[2], m_home_stand[3], m_home_stand[4], m_home_stand[5]); 
    }

    // m_home_stand = {0.1386, 0.25, -0.912, 0.1386, -0.25, -0.912};

    RCLCPP_DEBUG(this->get_logger(), COLOR_GREEN "Cartesian node configured! " RESET); 

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GaitPlanningCartesianNode::on_activate(const rclcpp_lifecycle::State &state) {
    
    (void) state;
    m_active = true;  
    m_iks_foot_positions_publisher->on_activate(); 
    m_interpolated_bezier_visualization_publisher_rviz->on_activate();
    RCLCPP_INFO(this->get_logger(),  "Cartesian node " COLOR_GREEN "activated!" RESET); 
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GaitPlanningCartesianNode::on_deactivate(const rclcpp_lifecycle::State &state) {
    
    (void) state; 
    m_active = false; 
    m_iks_foot_positions_publisher->on_deactivate(); 
    m_interpolated_bezier_visualization_publisher_rviz->on_deactivate(); 
    RCLCPP_INFO(this->get_logger(), "Cartesian node " COLOR_RED "deactivated!" RESET); 
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GaitPlanningCartesianNode::on_cleanup(const rclcpp_lifecycle::State &state) {
    
    (void) state; 
    m_iks_foot_positions_publisher.reset();
    m_interpolated_bezier_visualization_publisher_rviz.reset();   
    RCLCPP_DEBUG(this->get_logger(), "Cartesian node cleaned up!"); 

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GaitPlanningCartesianNode::on_shutdown(const rclcpp_lifecycle::State &state) {
    (void) state; 
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


void GaitPlanningCartesianNode::currentModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "Received current mode: " COLOR_PERIWINKLE "%s" RESET " with node_type" COLOR_PERIWINKLE " %s" RESET, toString(static_cast<ExoMode>(msg->mode)).c_str(), msg->node_type.c_str()); 
    // RCLCPP_INFO(this->get_logger(), "Received previous mode " COLOR_PERIWINKLE "%s" RESET, toString(static_cast<ExoMode>(msg->previous_mode)).c_str()); 
    // RCLCPP_INFO(this->get_logger(), "State of node is %s%s%s", 
    // m_active ? COLOR_GREEN : COLOR_RED, 
    // m_active ? "ACTIVE" : "INACTIVE", 
    // RESET);
    if (m_active){
        m_gait_planning.setPreviousGaitType((ExoMode)msg->previous_mode);  
        m_gait_planning.setGaitType((ExoMode)msg->mode);

    if ((ExoMode)msg->mode == ExoMode::Descending){
        m_single_execution_done = false; 
    }
        publishFootPositions(); 
    } else {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "not active"); 
    }
}

void GaitPlanningCartesianNode::currentExoJointStateCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg){
    // RCLCPP_INFO(get_logger(), "Received current foot positions");

    if (m_active){
        GaitPlanning::XYZFootPositionArray new_left_foot_position = {msg->body_ankle_pose[0].position.x, msg->body_ankle_pose[0].position.y, msg->body_ankle_pose[0].position.z};
        GaitPlanning::XYZFootPositionArray new_right_foot_position = {msg->body_ankle_pose[1].position.x, msg->body_ankle_pose[1].position.y, msg->body_ankle_pose[1].position.z};
        m_gait_planning.setFootPositions(new_left_foot_position, new_right_foot_position); 
        m_desired_footpositions_msg->header = msg->header;
        if (m_current_trajectory.empty()){
            m_gait_planning.setStanceFoot(msg->next_stance_leg); 
            if (m_gait_planning.getGaitType() == ExoMode::LargeWalk || m_gait_planning.getGaitType() == ExoMode::SmallWalk){
                RCLCPP_INFO(this->get_logger(), "Current stance foot is %s",
            (m_gait_planning.getCurrentStanceFoot() == 1 ? "left foot" : (m_gait_planning.getCurrentStanceFoot() == 2 ? "right foot" : "both feet")));
            }
        }
        publishFootPositions();
    } else {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "not active"); 
    }
}

void GaitPlanningCartesianNode::MPCCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "Received footsteps from MPC"); 

    m_left_foot_offset = m_gait_planning.getCurrentLeftFootPos(); 
    m_right_foot_offset = m_gait_planning.getCurrentRightFootPos(); 
    RCLCPP_DEBUG(this->get_logger(), "Current left foot stance: %f, %f, %f", m_left_foot_offset[0], m_left_foot_offset[1], m_left_foot_offset[2]); 
    RCLCPP_DEBUG(this->get_logger(), "Current right foot stance: %f, %f, %f", m_right_foot_offset[0], m_right_foot_offset[1], m_right_foot_offset[2]); 

    if (!m_current_trajectory.empty()){
        // wait until trajectory is finished
    } else if (m_current_trajectory.empty()){

        //Remove duplicates from PoseArray message to identify the two desired footsteps

        std::set<geometry_msgs::msg::Pose, GaitPlanningCartesianNode::PoseXComparator> final_feet(msg->poses.begin(), msg->poses.end());

        geometry_msgs::msg::Pose foot_pos = *final_feet.begin(); 
        auto second_foot = next(final_feet.begin(), 1); 

        if (!m_variable_first_step_done){
            RCLCPP_DEBUG(this->get_logger(), "Publishing first step"); 
            // first step 
            if (foot_pos.position.y > second_foot->position.y){
            // left foot 
                // m_variable_distance = foot_pos.position.x - m_left_foot_offset[0]; 
                m_gait_planning.setVariableDistance(foot_pos.position.x - m_left_foot_offset[0]); 
                m_variable_walk_swing_leg = 0; 
                RCLCPP_INFO(this->get_logger(), "Going to send a left swing foot!"); 
            } else if (foot_pos.position.y < second_foot->position.y){
            // right foot 
                // m_variable_distance = foot_pos.position.x - m_right_foot_offset[0]; 
                m_gait_planning.setVariableDistance(foot_pos.position.x - m_right_foot_offset[0]); 
                m_variable_walk_swing_leg = 1; 
                RCLCPP_INFO(this->get_logger(), "Going to send a right swing foot!"); 
            }

            RCLCPP_INFO(this->get_logger(), "Distance to be interpolated: %f", m_gait_planning.getVariableDistance()); 
            m_current_trajectory.clear(); 
            m_current_trajectory = m_gait_planning.variableFirstStepTrajectory(m_gait_planning.getVariableDistance()); 
            RCLCPP_DEBUG(this->get_logger(), "Trajectory interpolated with size %d, %d", m_current_trajectory.size(), m_current_trajectory[0].size()); 
            m_variable_first_step_done = true; 
            RCLCPP_DEBUG(this->get_logger(), "First step done!"); 
           
        } else if (m_variable_first_step_done){
            RCLCPP_DEBUG(this->get_logger(), "Publishing full steps"); 
            // full steps 
            if (foot_pos.position.y > second_foot->position.y){
            // left foot 
                // m_variable_distance = foot_pos.position.x - m_left_foot_offset[0]; 
                m_gait_planning.setVariableDistance(foot_pos.position.x - m_left_foot_offset[0]); 
                m_variable_walk_swing_leg = 0; 
                RCLCPP_INFO(this->get_logger(), "Going to send a left swing foot!"); 
            } else if (foot_pos.position.y < second_foot->position.y){
            // right foot 
                // m_variable_distance = foot_pos.position.x - m_right_foot_offset[0];
                m_gait_planning.setVariableDistance(foot_pos.position.x - m_right_foot_offset[0]);  
                m_variable_walk_swing_leg = 1; 
                RCLCPP_INFO(this->get_logger(), "Going to send a right swing foot!"); 
            }

            RCLCPP_INFO(this->get_logger(), "Distance to be interpolated: %f", m_gait_planning.getVariableDistance()); 
            m_current_trajectory.clear(); 
            m_current_trajectory = m_gait_planning.variableFullStepTrajectory(m_gait_planning.getVariableDistance()); 
        }

        /**
         * Below is code to visualize a posearray in RViz for MPC visualization.
         **/ 
        for (auto element : m_current_trajectory){
            m_pose->position.x = static_cast<float>(element[0]) + m_home_stand[m_variable_walk_swing_leg == 1 ? 3 : 0];
            m_pose->position.y = m_home_stand[m_variable_walk_swing_leg == 1 ? 4 : 1];
            m_pose->position.z = static_cast<float>(element[1]) + m_home_stand[m_variable_walk_swing_leg == 1 ? 5 : 2];
            m_visualization_msg_rviz->poses.push_back(*m_pose); 
        }
        m_visualization_msg_rviz->header.frame_id = "backpack"; 
        m_visualization_msg_rviz->header.stamp = this->now(); 
        m_interpolated_bezier_visualization_publisher_rviz->publish(*m_visualization_msg_rviz); 
        m_visualization_msg_rviz->poses.clear(); 
        RCLCPP_DEBUG(this->get_logger(), "Visualization msg filled and sent "); 


        publishFootPositions(); 

    }

}

void GaitPlanningCartesianNode::setFootPositionsMessage(double left_x, double left_y, double left_z, 
                                        double right_x, double right_y, double right_z) 
{
    m_desired_footpositions_msg->left_foot_position.x = left_x;
    m_desired_footpositions_msg->left_foot_position.y = left_y;
    m_desired_footpositions_msg->left_foot_position.z = left_z; 
    m_desired_footpositions_msg->right_foot_position.x = right_x; 
    m_desired_footpositions_msg->right_foot_position.y = right_y;
    m_desired_footpositions_msg->right_foot_position.z = right_z;
}

void GaitPlanningCartesianNode::finishCurrentTrajectory(){
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 100, "Finishing current trajectory before standing."); 
    GaitPlanning::XZFeetPositionsArray current_step = m_current_trajectory.front();
    m_current_trajectory.erase(m_current_trajectory.begin());
    if (m_gait_planning.getCurrentStanceFoot() & 0b1 || m_variable_walk_swing_leg == 1){
        // 01 is left stance leg, 11 is both, 00 is neither and 10 is right. 1 as last int means left or both. 
        setFootPositionsMessage(current_step[2]+m_home_stand[0], m_home_stand[1], current_step[3]+m_home_stand[2], 
                        current_step[0]+m_home_stand[3], m_home_stand[4], current_step[1]+m_home_stand[5]);
    } else if (m_gait_planning.getCurrentStanceFoot() & 0b10 || m_variable_walk_swing_leg == 0){
        // 10 is right stance leg
        setFootPositionsMessage(current_step[0]+m_home_stand[0], m_home_stand[1], current_step[1]+m_home_stand[2], 
                        current_step[2]+m_home_stand[3], m_home_stand[4], current_step[3]+m_home_stand[5]);
    }
    m_iks_foot_positions_publisher->publish(*m_desired_footpositions_msg);
}

void GaitPlanningCartesianNode::publishIncrements(){
    // RCLCPP_INFO(this->get_logger(), "publishing increment number %d", m_home_stand_trajectory.size()); 
    std::array<double, 6> current_step = m_home_stand_trajectory.front();
    // RCLCPP_INFO(this->get_logger(), "current step: %f, %f, %f, %f, %f, %f", current_step[0], current_step[1], current_step[2], current_step[3], current_step[4], current_step[5]); 
    m_home_stand_trajectory.erase(m_home_stand_trajectory.begin());   
    setFootPositionsMessage(current_step[0], current_step[1], current_step[2], current_step[3], current_step[4], current_step[5]);
    m_iks_foot_positions_publisher->publish(*m_desired_footpositions_msg);
}




void GaitPlanningCartesianNode::stepClose(){
    RCLCPP_DEBUG(this->get_logger(), "Calling step close trajectory with mode: %s", toString(static_cast<ExoMode>(m_gait_planning.getPreviousGaitType())).c_str());
    m_current_trajectory = m_gait_planning.getTrajectory();

    // Hier trajectory publishen naar visualizer, depending on previous gait type 
    switch (m_gait_planning.getPreviousGaitType()){
        case ExoMode::HighStep1: 
        case ExoMode::HighStep2:
        case ExoMode::HighStep3: 
            break; 
        case ExoMode::LargeWalk:
        case ExoMode::SmallWalk: 
        // case ExoMode::VariableWalk:
        //     break; 
        default: 
            break; 
    }

    RCLCPP_DEBUG(this->get_logger(), "Size of step close trajectory: %d", m_current_trajectory.size());
    m_gait_planning.setPreviousGaitType(ExoMode::Stand);
}


void GaitPlanningCartesianNode::calculateIncrements(){
    RCLCPP_WARN(this->get_logger(), "Calculating homestand trajectory in cartesian node "); 
    m_current_trajectory.clear(); 
    m_home_stand_trajectory.clear();  
    RCLCPP_INFO(this->get_logger(), "Incrementing to homestand"); 
    m_left_foot_offset = m_gait_planning.getCurrentLeftFootPos(); 
    m_right_foot_offset = m_gait_planning.getCurrentRightFootPos(); 
    m_initial_position = {m_left_foot_offset[0], m_left_foot_offset[1], m_left_foot_offset[2], m_right_foot_offset[0], m_right_foot_offset[1], m_right_foot_offset[2]}; 
    RCLCPP_DEBUG(this->get_logger(), "original position: %f, %f, %f, %f, %f, %f", m_initial_position[0], m_initial_position[1], m_initial_position[2], m_initial_position[3], m_initial_position[4], m_initial_position[5]); 
    for (unsigned i = 0; i < m_home_stand.size(); i++){
        m_increments.push_back((m_home_stand[i] - m_initial_position[i])/100); 
    }
    RCLCPP_DEBUG(this->get_logger(), "increments: %f, %f, %f, %f, %f, %f ", m_increments[0], m_increments[1], m_increments[2], m_increments[3], m_increments[4], m_increments[5]); 
    for (unsigned i = 0; i < 100; i++){
        for (unsigned i = 0; i < m_initial_position.size(); i++){
        m_initial_position[i] += m_increments[i]; 
        }
        m_home_stand_trajectory.push_back(m_initial_position); 
    }
    RCLCPP_DEBUG(this->get_logger(), "Calculated incremental steps, length of trajectory: %d", m_home_stand_trajectory.size()); 
    m_gait_planning.setPreviousGaitType(ExoMode::Stand); 
}

void GaitPlanningCartesianNode::publishHomeStand(){
    m_current_trajectory.clear();
    setFootPositionsMessage(m_home_stand[0], m_home_stand[1], m_home_stand[2], m_home_stand[3], m_home_stand[4], m_home_stand[5]);
    m_iks_foot_positions_publisher->publish(*m_desired_footpositions_msg);
    m_single_execution_done = false; 
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "Publishing homestand position.");
}

void GaitPlanningCartesianNode::processStand(){
    m_single_execution_done = false;
    if (!m_current_trajectory.empty()){
        finishCurrentTrajectory(); 
    } else if (!m_home_stand_trajectory.empty() && m_gait_planning.getPreviousGaitType() == ExoMode::BootUp){
        // Incrementing to homestand should only happen when coming from bootup mode. Due to switching between LCNs the incrementing trajectory was still stored in the cartesian node when exo had already transitioned to Stand. 
        RCLCPP_WARN(this->get_logger(), "homestand trajectory not empty"); 
        publishIncrements(); 
    } else {
        switch (m_gait_planning.getPreviousGaitType()){
            case ExoMode::LargeWalk :
            case ExoMode::SmallWalk :
            case ExoMode::HighStep1 :
            case ExoMode::HighStep2 :
            case ExoMode::HighStep3 :
            // case ExoMode::VariableWalk :
            //     stepClose(); 
            //     break; 

            case ExoMode::BootUp :
            // As the stand mode is executed from the joint angles node from now on, calculating increments is not necessary. This will only cause logic errors down the line. 
                // calculateIncrements(); 
                break; 

            default :
                publishHomeStand(); 
                break; 
        }
    }
}

void GaitPlanningCartesianNode::publishWalk(){
    if (m_current_trajectory.empty()) {
        m_current_trajectory = m_gait_planning.getTrajectory(); 
        RCLCPP_DEBUG(this->get_logger(), "Trajectory refilled!");
    } else {
        GaitPlanning::XZFeetPositionsArray current_step = m_current_trajectory.front();
        m_current_trajectory.erase(m_current_trajectory.begin());
        if (m_gait_planning.getCurrentStanceFoot() & 0b1){
            // 01 is left stance leg, 11 is both, 00 is neither and 10 is right. 1 as last int means left or both. 
            setFootPositionsMessage(current_step[2]+m_home_stand[0], m_home_stand[1], current_step[3]+m_home_stand[2], 
                            current_step[0]+m_home_stand[3], m_home_stand[4], current_step[1]+m_home_stand[5]);
        } else if (m_gait_planning.getCurrentStanceFoot() & 0b10){
            // 10 is right stance leg
            setFootPositionsMessage(current_step[0]+m_home_stand[0], m_home_stand[1], current_step[1]+m_home_stand[2], 
                            current_step[2]+m_home_stand[3], m_home_stand[4], current_step[3]+m_home_stand[5]);
        }
        m_iks_foot_positions_publisher->publish(*m_desired_footpositions_msg);
    }
}

void GaitPlanningCartesianNode::publishHeightGaits(){
    if (m_current_trajectory.empty() && !m_single_execution_done){
        m_current_trajectory = m_gait_planning.getTrajectory(); 
        m_single_execution_done = true;

        RCLCPP_DEBUG(this->get_logger(), "Height trajectory filled, size of current trajectory: %d", m_current_trajectory.size());
    }
    else if (m_current_trajectory.empty() && m_single_execution_done){
        setFootPositionsMessage(m_home_stand[0], m_home_stand[1], m_home_stand[2], m_home_stand[3], m_home_stand[4], m_home_stand[5]);
        m_iks_foot_positions_publisher->publish(*m_desired_footpositions_msg);
    }
    else {
        GaitPlanning::XZFeetPositionsArray current_step = m_current_trajectory.front();
        m_current_trajectory.erase(m_current_trajectory.begin());
        setFootPositionsMessage(current_step[2]+m_home_stand[0], m_home_stand[1], current_step[3] + m_home_stand[2], 
                        current_step[0]+m_home_stand[3], m_home_stand[4], current_step[1] + m_home_stand[5]);
        m_iks_foot_positions_publisher->publish(*m_desired_footpositions_msg);
    }
}

void GaitPlanningCartesianNode::publishVariableWalk(){
    // TODO: figure out if offset is still needed. 
    if (!m_current_trajectory.empty()){
        GaitPlanning::XZFeetPositionsArray current_step = m_current_trajectory.front();
        m_current_trajectory.erase(m_current_trajectory.begin());
        if (m_variable_walk_swing_leg == 1){
            // right swing leg 
            setFootPositionsMessage(current_step[2]+m_home_stand[0], m_home_stand[1], current_step[3]+m_home_stand[2], 
                            current_step[0]+m_home_stand[3], m_home_stand[4], current_step[1]+m_home_stand[5]);
        } else if (m_variable_walk_swing_leg == 0){
            // left swing leg 
            setFootPositionsMessage(current_step[0]+m_home_stand[0], m_home_stand[1], current_step[1]+m_home_stand[2], 
                            current_step[2]+m_home_stand[3], m_home_stand[4], current_step[3]+m_home_stand[5]);
        }
        m_iks_foot_positions_publisher->publish(*m_desired_footpositions_msg);
    }
}
 
void GaitPlanningCartesianNode::publishFootPositions(){
    switch (m_gait_planning.getGaitType()){
        case ExoMode::Stand :
            processStand(); 
            break;

        case ExoMode::BootUp :
            RCLCPP_DEBUG(this->get_logger(), "BootUp mode entered, waiting for new mode."); 
            break;
        
        case ExoMode::LargeWalk :
        case ExoMode::SmallWalk :
            publishWalk(); 
            break;

        case ExoMode::VariableStep : 
            if (m_current_trajectory.empty()){
                setFootPositionsMessage(m_home_stand[0], m_home_stand[1], m_home_stand[2], m_home_stand[3], m_home_stand[4], m_home_stand[5]);
                m_iks_foot_positions_publisher->publish(*m_desired_footpositions_msg);
            }
            else { 
                GaitPlanning::XZFeetPositionsArray current_step = m_current_trajectory.front();
                m_current_trajectory.erase(m_current_trajectory.begin());
                setFootPositionsMessage(current_step[2]+m_home_stand[0], m_home_stand[1], current_step[3] + m_home_stand[2], 
                                current_step[0]+m_home_stand[3], m_home_stand[4], current_step[1] + m_home_stand[5]);
                m_iks_foot_positions_publisher->publish(*m_desired_footpositions_msg);
            } 
            break;
        
        case ExoMode::HighStep1 :
        case ExoMode::HighStep2 :
        case ExoMode::HighStep3 :
        case ExoMode::Ascending :
        case ExoMode::Descending :
            publishHeightGaits(); 
            break;
        
        // case ExoMode::VariableWalk :
        //     publishVariableWalk(); 
        //     break; 

        default :
            break;
    }
}

std::vector<double> GaitPlanningCartesianNode::parseHomestandYAML(const std::string& file_path){

    std::vector<double> values;
    
    try {
        YAML::Node config = YAML::LoadFile(file_path);
        YAML::Node cartesian = config["cartesian"];
        if (cartesian && cartesian.IsSequence()) {
            for (const auto& value : cartesian) {
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
    std::shared_ptr<GaitPlanningCartesianNode> gait_planning_cartesian_node = std::make_shared<GaitPlanningCartesianNode>();
    executor.add_node(gait_planning_cartesian_node->get_node_base_interface()); 
    executor.spin(); 
    rclcpp::shutdown(); 

    return 0; 
}

