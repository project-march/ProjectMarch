
#pragma once
#include "rclcpp/rclcpp.hpp"
#include "march_input_device/input_device.hpp"
#include "std_msgs/msg/int32.hpp"
#include "march_shared_msgs/msg/exo_mode_array.hpp"
#include "march_shared_msgs/srv/get_exo_mode_array.hpp"

class BluetoothInputDeviceNode : public rclcpp::Node {
public:
    explicit BluetoothInputDeviceNode();
    ~BluetoothInputDeviceNode();
    void spin();

private:
    rclcpp::Client<march_shared_msgs::srv::GetExoModeArray>::SharedPtr m_get_exo_mode_array_client;
    
    void sendNewMode(const exoMode& desired_mode);

    void createSocket();
    void receiveData();
    void processData(char* buffer, ssize_t bytesRead);
    



    IPD m_ipd;

    int m_bluetooth_socket;


};