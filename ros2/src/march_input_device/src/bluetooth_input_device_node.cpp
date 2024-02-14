#include "march_input_device/bluetooth_input_device_node.hpp"
#include "march_input_device/input_device.hpp"
#include <cstdlib>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <unistd.h>
using std::placeholders::_1;

std::array<exoMode, 10> all_possible_modes = {exoMode::Sit, exoMode::Stand, exoMode::Walk, exoMode::BootUp, exoMode::Sideways, exoMode::LargeWalk, exoMode::SmallWalk, exoMode::Ascending, exoMode::Descending, exoMode::VariableWalk};

BluetoothInputDeviceNode::BluetoothInputDeviceNode()
  : Node("march_input_device_node"),
  m_ipd (IPD())
{
    m_get_exo_mode_array_client = 
        create_client<march_shared_msgs::srv::GetExoModeArray>("get_exo_mode_array");
    while (!m_get_exo_mode_array_client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_WARN(this->get_logger(), "Waiting for service get_exo_mode_array to become available...");
    }

    RCLCPP_INFO(this->get_logger(), "Connected to service get_exo_mode_array");

    m_ipd.setCurrentMode(exoMode::BootUp);
    sendNewMode(m_ipd.getCurrentMode());

    createSocket();

    RCLCPP_INFO(this->get_logger(), "Exiting constructor");
}

BluetoothInputDeviceNode::~BluetoothInputDeviceNode()
{
    close(m_bluetooth_socket);
    RCLCPP_INFO(this->get_logger(), "Socket succesfully closed, destructing node");
}

void BluetoothInputDeviceNode::createSocket()
{
    struct sockaddr_rc addr = { 0 };
    addr.rc_family = AF_BLUETOOTH;
    str2ba("64:5D:F4:14:B4:7E", &addr.rc_bdaddr);
    addr.rc_channel = (uint8_t) 6;

    // Open a socket to the local Bluetooth adapter
    m_bluetooth_socket = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
    if (m_bluetooth_socket < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error opening socket: %s", strerror(errno));
        return;
    }

    // Connect to the remote device
    int result = connect(m_bluetooth_socket, (struct sockaddr *)&addr, sizeof(addr));
    if (result == -1) {
        RCLCPP_ERROR(this->get_logger(), "Error connecting to remote device: %s", strerror(errno));
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Connected to remote device successfully");
}

void BluetoothInputDeviceNode::receiveData()
{
    char buffer[1024] = { 0 }; // This 
    ssize_t bytesRead = read(m_bluetooth_socket, buffer, sizeof(buffer) - 1); // This is blocking so will not pass unless there is something to read
    if (bytesRead == -1) {
        RCLCPP_ERROR(this->get_logger(), "Error reading from socket:  %s", strerror(errno));
        return;
    }

    // Process the received data
    processData(buffer, bytesRead);
}

void BluetoothInputDeviceNode::processData(char* buffer, ssize_t bytesRead)
{
    int received_int;
    memcpy(&received_int, buffer, sizeof(received_int));

    RCLCPP_INFO(this->get_logger(), "Received: %d", received_int);

    m_ipd.setCurrentMode(static_cast<exoMode>(received_int));
    sendNewMode(m_ipd.getCurrentMode());
}

void BluetoothInputDeviceNode::sendNewMode(const exoMode& desired_mode)
{
    auto request = std::make_shared<march_shared_msgs::srv::GetExoModeArray::Request>();
    request->desired_mode.mode = static_cast<int32_t>(desired_mode);

    auto result_future = m_get_exo_mode_array_client->async_send_request(request);


    // Wait for the result
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), result_future) ==
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        auto result = result_future.get();
        // Do something with the result
        std::set<exoMode> exo_modes_set;
        for (const auto& exo_mode_msg : result->mode_array.modes) {
            exo_modes_set.insert(static_cast<exoMode>(exo_mode_msg.mode));
        }
        m_ipd.setAvailableModes(exo_modes_set);

    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service GetExoModeArray");
    }
}

void BluetoothInputDeviceNode::spin()
{
    while (rclcpp::ok()) {
        receiveData();
    }
}

int main(int argc, char *argv[]) 
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<BluetoothInputDeviceNode>();
  node->spin();

  rclcpp::shutdown();
  return 0;
}
