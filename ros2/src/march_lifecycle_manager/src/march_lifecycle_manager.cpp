/* Authors: Martijn Habers M9

This node is used to manage the lifecycle of the lifecycle nodes

*/

#include "march_lifecycle_manager/march_lifecycle_manager.hpp"

template <typename FutureT, typename WaitTimeT>
std::future_status wait_for_result(FutureT& future, WaitTimeT time_to_wait)
{
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;
  do
  {
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0)){break;}
    status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
  } while (rclcpp::ok() && status != std::future_status::ready);
  return status;
}

LifecycleManager::LifecycleManager() : Node("lifecycle_manager")
{
  RCLCPP_INFO(this->get_logger(), "Lifecycle Manager Node started");
}

void LifecycleManager::init()
{
  client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(node_get_state_topic);
  client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(node_change_state_topic);
}

unsigned int LifecycleManager::get_state(std::chrono::seconds time_out = std::chrono::seconds(3))
{
  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

  if (!client_get_state_->wait_for_service(time_out))
  {
    RCLCPP_ERROR(get_logger(), "Service %s is not available.", client_get_state_->get_service_name());
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }

  auto future_result = client_get_state_->async_send_request(request);
  auto future_status = wait_for_result(future_result, time_out);

  if (future_status != std::future_status::ready)
  {
    RCLCPP_ERROR(get_logger(), "Server time out while getting current state for node %s", lifecycle_node);
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }

  if (future_result.get())
  {
    RCLCPP_INFO(get_logger(), "Node %s has current state %s.", lifecycle_node,
                future_result.get()->current_state.label.c_str());
    return future_result.get()->current_state.id;
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "Failed to get current state for node %s", lifecycle_node);
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }
}

bool
  change_state(std::uint8_t transition, std::chrono::seconds time_out = 3s)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    if (!client_change_state_->wait_for_service(time_out)) {
      RCLCPP_ERROR(
        get_logger(),
        "Service %s is not available.",
        client_change_state_->get_service_name());
      return false;
    }

    // We send the request with the transition we want to invoke.
    auto future_result = client_change_state_->async_send_request(request);

    // Let's wait until we have the answer from the node.
    // If the request times out, we return an unknown state.
    auto future_status = wait_for_result(future_result, time_out);

    if (future_status != std::future_status::ready) {
      RCLCPP_ERROR(
        get_logger(), "Server time out while getting current state for node %s", lifecycle_node);
      return false;
    }

    // We have an answer, let's print our success.
    if (future_result.get()->success) {
      RCLCPP_INFO(
        get_logger(), "Transition %d successfully triggered.", static_cast<int>(transition));
      return true;
    } else {
      RCLCPP_WARN(
        get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(transition));
      return false;
    }
  }

static constexpr char const* node_get_state_topic = "gait_planning_node/get_state";
static constexpr char const* node_change_state_topic = "gait_planning_node/change_state";


// mains

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto lc_client = std::make_shared<LifecycleManager>();
  lc_client->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(lc_client);

  std::shared_future<void> script = std::async(
    std::launch::async,
    std::bind(callee_script, lc_client));
  exe.spin_until_future_complete(script);

  rclcpp::shutdown();

  return 0;
}
