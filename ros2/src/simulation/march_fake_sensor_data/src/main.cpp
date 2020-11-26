#include "rclcpp/rclcpp.hpp"
#include "march_fake_sensor_data/FakeTemperatureData.hpp"
#include <vector>
#include <memory>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Create a FakeTemperatureNode instance.
    std::vector<float> autoregression_weights { 0.1, 0.1, 0.1, 0.15, 0.15, 0.2, 0.2 };
    auto temperature = std::make_shared<FakeTemperatureDataNode> (
        "march_fake_temperature_data",
        std::move(autoregression_weights)
    );

    // Execute the temperature node with a single-threaded executor.
    rclcpp::spin(temperature);
    rclcpp::shutdown();
    return 0;
      /*
  ros::init(argc, argv, "march_fake_sensor_data");
  ros::nodehandle n;
  ros::rate rate(10);

  ros::param::get("~min_temperature", min_temperature);
  ros::param::get("~max_temperature", max_temperature);

  int count = 0;
  while (!n.hasparam("/march/joint_names"))
  {
    ros::duration(0.5).sleep();
    count++;
    if (count > 10)
    {
      ros_error("failed to read the joint_names from the parameter server.");
      throw std::runtime_error("failed to read the joint_names from the parameter server.");
    }
  }

  n.getparam("/march/joint_names", sensor_names);

  // initialise autoregression variables.
  latest_temperatures = { 0, 0, 0, 0, 0, 0, 0 };
  ar_values = { 0.1, 0.1, 0.1, 0.15, 0.15, 0.2, 0.2 };

  // create a publisher for each sensor
  for (std::string sensor_name : sensor_names)
  {
    ros::publisher temperature_pub =
        n.advertise<sensor_msgs::temperature>(createtopicname("/march/temperature", sensor_name.c_str()), 1000);
    temperature_publishers.push_back(temperature_pub);
  }

  // make the temperature values dynamic reconfigurable
  dynamic_reconfigure::server<march_fake_sensor_data::temperaturesconfig> server;
  dynamic_reconfigure::server<march_fake_sensor_data::temperaturesconfig>::callbacktype f;
  server.setcallback(boost::bind(&temperatureconfigcallback, _1, _2));

  while (ros::ok())
  {
    // loop through all publishers
    for (ros::publisher temperature_pub : temperature_publishers)
    {
      publishtemperature(temperature_pub);
    }

    rate.sleep();
    ros::spinonce();
  }
*/
  return 0;
}
