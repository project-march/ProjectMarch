#ifndef ODRIVE_HPP_
#define ODRIVE_HPP_

#include <cstdint>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <getopt.h>
#include <iostream>
#include <string>
#include <vector>
#include <libusb-1.0/libusb.h>
#include <string>
#include <fstream>
#include <jsoncpp/json/json.h>

#include "ros/ros.h"
#include "odrive_endpoint.h"
#include "odrive_enums.h"
#include "march_hardware/motor_controller/motor_controller.h"

#define ODRIVE_OK 0;
#define ODRIVE_ERROR 1;



static constexpr double PI_2 = 2 * M_PI;

namespace march
{
typedef struct odrive_json_object
{
  int id;
  std::string name;
  std::string type;
  std::string access;
} odrive_json_object;

class Odrive : public MotorController
{
public:
  /**
   * Initialize the odrive with specified axis
   */
  Odrive(const std::string& axis_number, std::shared_ptr<OdriveEndpoint> odrive_endpoint, bool import_json = true);

  /**
   * Import the odrive json structure from the odrive
   */
  int importOdriveJson();

  /**
   * Check if given value type matched value type of odrive variable
   */
  template <typename TT>
  int validateType(const odrive_json_object& json_object, TT& value);

  /**
   * Read parameter from the odrive object
   */
  template <typename TT>
  int read(const std::string& parameter_name, TT& value);

  /**
   * Write parameter to the odrive object
   */
  template <typename TT>
  int write(const std::string& parameter_name, TT& value);

  /**
   * Execute function on the odrive object
   */
  int function(const std::string& function_name);

  /**
   * Set configurations in the Json file to the Odrive
   */
  int setConfigurations(const std::string& configuration_json_path);

  std::string axis_number;

protected:
  float readMotorControllerVoltage();
  float readMotorCurrent();
  float readMotorVoltage();

  int32_t readAxisError();
  int32_t readAxisMotorError();
  int32_t readAxisEncoderError();
  int32_t readAxisControllerError();

  int readAngleCountsAbsolute();
  double readVelocityRadAbsolute();

  double readAngleCountsIncremental();
  double readVelocityRadIncremental();
  std::string create_command(std::string command_name);

private:
  int json_string_read(const Json::Value& json_parameter_object);

  int json_string_write(const Json::Value& json_parameter_object);

  static std::vector<std::string> split_string(const std::string& str, char delimiter = '.');

  odrive_json_object getJsonObject(const std::string& parameter_name);

  Json::Value odrive_json_;
  Json::Value odrive_configuration_json_;

  std::shared_ptr<OdriveEndpoint> odrive_endpoint_;
};
}  // namespace march
#endif
