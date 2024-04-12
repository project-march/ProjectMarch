#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

using namespace std::chrono_literals; 

static constexpr char const * talkerNode = "gait_planning_angles_node"; 
static constexpr char const * nodeGetStateTopic = "gait_planning_angles_node/get_state"; 
static constexpr char const * nodeChangeStateTopic = "gait_planning_angles_node/change_state"; 

class ServiceClient : public rclcpp::Node 
{
    public: 

    explicit ServiceClient();

    template <typename FutureT, typename WaitTimeT> std::future_status waitForResult(FutureT &future, WaitTimeT timeout); 
    void call_script(std::shared_ptr<ServiceClient> service_client); 

    unsigned int getState(std::chrono::seconds timeout = 3s);
    bool changeState(std::uint8_t transition, std::chrono::seconds timeout = 3s); 


    private: 
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> m_client_get_state; 
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> m_client_change_state; 

}; 