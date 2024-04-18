#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "march_shared_msgs/msg/exo_mode.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "../../march_mode_machine/include/march_mode_machine/exo_mode.hpp"
#include <string>


using namespace std::chrono_literals; 

static constexpr char const * anglesNode = "gait_planning_angles_node"; 
static constexpr char const * cartesianNode = "gait_planning_cartesian_node"; 
static constexpr char const * anglesNodeGetStateTopic = "gait_planning_angles_node/get_state"; 
static constexpr char const * anglesNodeChangeStateTopic = "gait_planning_angles_node/change_state"; 
static constexpr char const * cartesianNodeGetStateTopic = "gait_planning_cartesian_node/get_state"; 
static constexpr char const * cartesianNodeChangeStateTopic = "gait_planning_cartesian_node/change_state"; 

class ServiceClient : public rclcpp::Node 
{
    public: 

    explicit ServiceClient();

    template <typename FutureT, typename WaitTimeT> std::future_status waitForResult(FutureT &future, WaitTimeT timeout); 
    void call_script(std::shared_ptr<ServiceClient> service_client); 


    unsigned int getAnglesState(std::chrono::seconds timeout = 3s);
    bool changeAnglesState(std::uint8_t transition, std::chrono::seconds timeout = 1s); 
    unsigned int getCartesianState(std::chrono::seconds timeout = 3s);
    bool changeCartesianState(std::uint8_t transition, std::chrono::seconds timeout = 1s);

    void modeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg); 

    void publishModeToGaitPlanning(const march_shared_msgs::msg::ExoMode::SharedPtr msg); 

    private: 
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> m_angles_client_get_state; 
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> m_angles_client_change_state;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> m_cartesian_client_get_state; 
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> m_cartesian_client_change_state; 

    rclcpp::Subscription<march_shared_msgs::msg::ExoMode>::SharedPtr m_mode_subscriber;
    rclcpp::Publisher<march_shared_msgs::msg::ExoMode>::SharedPtr m_gaitplanning_mode_publisher; 

    bool m_cartesian_active; 
    bool m_angles_active; 
}; 