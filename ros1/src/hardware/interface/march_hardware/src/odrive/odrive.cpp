#include "march_hardware/motor_controller/odrive/odrive.h"

namespace march
{
Odrive::Odrive(const std::string& axis_number, std::shared_ptr<OdriveEndpoint> odrive_endpoint, bool import_json)
{
  this->axis_number = axis_number;
  this->odrive_endpoint_ = std::move(odrive_endpoint);

  if (import_json)
  {
    if (this->importOdriveJson())
    {
      ROS_ERROR("Odrive %s error getting JSON", odrive_endpoint_->getSerialNumber().c_str());
    }
  }
}

std::vector<std::string> Odrive::split_string(const std::string& string_name, char delimiter)
{
  std::vector<std::string> split_string;

  std::stringstream ss(string_name);
  std::string token;

  while (std::getline(ss, token, delimiter))
  {
    split_string.push_back(token);
  }

  return split_string;
}

odrive_json_object Odrive::getJsonObject(const std::string& parameter_name)
{
  odrive_json_object json_object;
  std::vector<std::string> parameter_list = this->split_string(parameter_name);

  Json::Value odrive_json_member_object;
  Json::Value odrive_json_member_list = this->odrive_json_;

  for (std::string& split_parameter_name : parameter_list)
  {
    for (auto& json_parameter : odrive_json_member_list)
    {
      if (json_parameter["name"].asCString() == split_parameter_name)
      {
        odrive_json_member_object = json_parameter;
        odrive_json_member_list = json_parameter["members"];
        break;
      }
    }
  }

  if (!odrive_json_member_object or odrive_json_member_object["name"].asCString() != parameter_list.back())
  {
    json_object.id = -1;
    ROS_ERROR("Odrive object with name %s is not found in parsed json file", parameter_name.c_str());
  }
  else
  {
    json_object.id = odrive_json_member_object["id"].asInt();
    json_object.name = odrive_json_member_object["name"].asString();
    json_object.type = odrive_json_member_object["type"].asString();
    json_object.access = odrive_json_member_object["access"].asString();
  }

  return json_object;
}

int Odrive::importOdriveJson()
{
  commBuffer rx;
  commBuffer tx;

  int len;
  int address = 0;

  std::string json;

  do
  {
    this->odrive_endpoint_->endpointRequest(0, rx, len, tx, true, 512, true, address);
    address = address + len;
    json.append((const char*)&rx[0], (size_t)len);
  } while (len > 0);

  Json::Reader reader;
  bool res = reader.parse(json, this->odrive_json_);

  if (!res)
  {
    return ODRIVE_ERROR;
  }
  return ODRIVE_OK;
}

int Odrive::function(const std::string& function_name)
{
  odrive_json_object json_object = this->getJsonObject(function_name);

  if (json_object.id == -1)
  {
    return ODRIVE_ERROR;
  }

  if (json_object.type != "function")
  {
    ROS_ERROR("Error: Given parameter %s is not a function on the Odrive", function_name.c_str());
    return ODRIVE_ERROR;
  }

  this->odrive_endpoint_->execFunc(json_object.id);
  return 0;
}

template <typename TT>
int Odrive::validateType(const odrive_json_object& json_object, TT& value)
{
//  std::cout << "----------" << std::endl;
//  std::cout << "Name: " << json_object.name << std::endl;
//  std::cout << "Type: " << json_object.type << std::endl;
//  std::cout << "Value: " << value << std::endl;
//  std::cout << "Sizeof value: " << sizeof(value) << std::endl;

  if (json_object.type == "float")
  {
    if (sizeof(value) != sizeof(float))
    {
      ROS_ERROR("Error value for %s is not float", json_object.name.c_str());
      return ODRIVE_ERROR;
    }
  }
  else if (json_object.type == "uint8")
  {
    if (sizeof(value) != sizeof(uint8_t))
    {
      ROS_ERROR("Error value for %s is not uint8_t", json_object.name.c_str());
      return ODRIVE_ERROR;
    }
  }
  else if (json_object.type == "uint16")
  {
    if (sizeof(value) != sizeof(uint16_t))
    {
      ROS_ERROR("Error value for %s is not uint16_t", json_object.name.c_str());
      return ODRIVE_ERROR;
    }
  }
  else if (json_object.type == "uint32")
  {
    if (sizeof(value) != sizeof(uint32_t))
    {
      ROS_ERROR("Error value for %s is not uint32_t", json_object.name.c_str());
      return ODRIVE_ERROR;
    }
  }
  else if (json_object.type == "uint64")
  {
    if (sizeof(value) != sizeof(uint64_t))
    {
      ROS_ERROR("Error value for %s is not uint64_t", json_object.name.c_str());
      return ODRIVE_ERROR;
    }
  }
  else if (json_object.type == "int8")
  {
    if (sizeof(value) != sizeof(short))
    {
      ROS_ERROR("Error value for %s is not short", json_object.name.c_str());
      return ODRIVE_ERROR;
    }
  }
  else if (json_object.type == "int16")
  {
    if (sizeof(value) != sizeof(short))
    {
      ROS_ERROR("Error value for %s is not short", json_object.name.c_str());
      return ODRIVE_ERROR;
    }
  }
  else if (json_object.type == "int32")
  {
    if (sizeof(value) != sizeof(int))
    {
      ROS_ERROR("Error value for %s is not int", json_object.name.c_str());
      return ODRIVE_ERROR;
    }
  }

  else if (json_object.type == "bool")
  {
    if (sizeof(value) != sizeof(bool))
    {
      ROS_ERROR("Error value for %s is not bool", json_object.name.c_str());
      return ODRIVE_ERROR;
    }
  }
  else
  {
    ROS_ERROR("Error: invalid type for %s", json_object.name.c_str());
    return ODRIVE_ERROR;
  }

  return ODRIVE_OK;
}

template <typename TT>
int Odrive::read(const std::string& parameter_name, TT& value)
{
  odrive_json_object json_object = this->getJsonObject(parameter_name);

  if (json_object.id == -1)
  {
    return ODRIVE_ERROR;
  }

  if (json_object.access.find('r') == std::string::npos)
  {
    ROS_ERROR("Error: invalid read access for %s", parameter_name.c_str());
    return ODRIVE_ERROR;
  }

  if (this->validateType(json_object, value) == 1)
  {
    return ODRIVE_ERROR;
  }

  return this->odrive_endpoint_->getData(json_object.id, value);
}

template <typename TT>
int Odrive::write(const std::string& parameter_name, TT& value)
{
  odrive_json_object json_object = this->getJsonObject(parameter_name);

  if (json_object.id == -1)
  {
    return ODRIVE_ERROR;
  }

  if (json_object.access.find('w') == std::string::npos)
  {
    ROS_ERROR("Error: invalid read access for %s", parameter_name.c_str());
    return ODRIVE_ERROR;
  }

  if (this->validateType(json_object, value) == 1)
  {
    return ODRIVE_ERROR;
  }

  return this->odrive_endpoint_->setData(json_object.id, value);
}

int Odrive::json_string_read(const Json::Value& json_parameter_object)
{
  std::string parameter_name = json_parameter_object["name"].asString();
  std::string type_name = json_parameter_object["type"].asString();
  Json::Value value = json_parameter_object["value"];

  if (type_name == "uint8")
  {
    uint8_t casted_value = value.asUInt();
    return this->read(parameter_name, casted_value);
  }
  else if (type_name == "uint16")
  {
    uint16_t casted_value = value.asUInt();
    return this->read(parameter_name, casted_value);
  }
  else if (type_name == "uint32")
  {
    uint32_t casted_value = value.asUInt();
    return this->read(parameter_name, casted_value);
  }
  else if (type_name == "int8")
  {
    int8_t casted_value = value.asUInt();
    return this->read(parameter_name, casted_value);
  }
  else if (type_name == "int16")
  {
    int16_t casted_value = value.asUInt();
    return this->read(parameter_name, casted_value);
  }
  else if (type_name == "int32")
  {
    int32_t casted_value = value.asUInt();
    return this->read(parameter_name, casted_value);
  }
  else if (type_name == "float")
  {
    float casted_value = value.asFloat();
    return this->read(parameter_name, casted_value);
  }
  else if (type_name == "bool")
  {
    bool casted_value = value.asBool();
    return this->read(parameter_name, casted_value);
  }
  else
  {
    ROS_ERROR("Error converting string for reading, invalid type %s", parameter_name.c_str());
    return ODRIVE_ERROR
  }
}

int Odrive::json_string_write(const Json::Value& json_parameter_object)
{
  std::string parameter_name = json_parameter_object["name"].asString();
  std::string type_name = json_parameter_object["type"].asString();
  Json::Value value = json_parameter_object["value"];

  if (type_name == "uint8")
  {
    uint8_t casted_value = value.asUInt();
    return this->write(parameter_name, casted_value);
  }
  else if (type_name == "uint16")
  {
    uint16_t casted_value = value.asUInt();
    return this->write(parameter_name, casted_value);
  }
  else if (type_name == "uint32")
  {
    uint32_t casted_value = value.asUInt();
    return this->write(parameter_name, casted_value);
  }
  else if (type_name == "int8")
  {
    int8_t casted_value = value.asInt();
    return this->write(parameter_name, casted_value);
  }
  else if (type_name == "int16")
  {
    int16_t casted_value = value.asInt();
    return this->write(parameter_name, casted_value);
  }
  else if (type_name == "int32")
  {
    int32_t casted_value = value.asInt();
    return this->write(parameter_name, casted_value);
  }
  else if (type_name == "float")
  {
    float casted_value = value.asFloat();
    return this->write(parameter_name, casted_value);
  }
  else if (type_name == "bool")
  {
    bool casted_value = value.asBool();
    return this->write(parameter_name, casted_value);
  }
  else
  {
    ROS_ERROR("Error converting string for writing, invalid type %s", parameter_name.c_str());
    return ODRIVE_ERROR
  }
}

int Odrive::setConfigurations(const std::string& configuration_json_path)
{
  std::ifstream cfg;
  std::string line, json;
  cfg.open(configuration_json_path, std::ios::in);

  if (cfg.is_open())
  {
    while (getline(cfg, line))
    {
      json.append(line);
    }
  }
  cfg.close();

  Json::Reader reader;
  bool res = reader.parse(json, this->odrive_configuration_json_);

  if (!res)
  {
    ROS_INFO("Error parsing odrive configuration, error");
    return ODRIVE_ERROR
  }

  for (auto& parameter : this->odrive_configuration_json_)
  {
    ROS_DEBUG("Setting %s to %s", parameter["name"].asString().c_str(), parameter["value"].asString().c_str());
    int result = this->json_string_write(parameter);

    if (result != LIBUSB_SUCCESS)
    {
      ROS_WARN("Setting %s to %s failed", parameter["name"].asString().c_str(), parameter["value"].asString().c_str());
      continue;
    }
  }

  uint8_t control_mode;
  std::string command_name_ = this->create_command(".controller.config.control_mode");
  if (this->read(command_name_, control_mode) == 1)
  {
    ROS_ERROR("Could not read control mode");
    return ODRIVE_ERROR;
  }
  else
  {
    ROS_INFO_STREAM("Control mode: " << control_mode);
  }

  return ODRIVE_OK
}

int32_t Odrive::readAxisError()
{
  uint32_t axis_error;
  std::string command_name_ = this->create_command(O_PM_AXIS_ERROR);
  if (this->read(command_name_, axis_error) == 1)
  {
    ROS_ERROR("Could not retrieve axis error");
    return ODRIVE_ERROR;
  }

  return axis_error;
}

int32_t Odrive::readAxisMotorError()
{
  uint64_t axis_motor_error;
  std::string command_name_ = this->create_command(O_PM_AXIS_MOTOR_ERROR);
  if (this->read(command_name_, axis_motor_error) == 1)
  {
    ROS_ERROR("Could not retrieve axis motor error");
    return ODRIVE_ERROR;
  }

  return axis_motor_error;
}

int32_t Odrive::readAxisEncoderError()
{
  uint16_t axis_encoder_error;
  std::string command_name_ = this->create_command(O_PM_AXIS_ENCODER_ERROR);
  if (this->read(command_name_, axis_encoder_error) == 1)
  {
    ROS_ERROR("Could not retrieve axis encoder error");
    return ODRIVE_ERROR;
  }

  return axis_encoder_error;
}

int32_t Odrive::readAxisControllerError()
{
  uint8_t axis_controller_error;
  std::string command_name_ = this->create_command(O_PM_AXIS_CONTROLLER_ERROR);
  if (this->read(command_name_, axis_controller_error) == 1)
  {
    ROS_ERROR("Could not retrieve axis controller error");
    return ODRIVE_ERROR;
  }

  return axis_controller_error;
}

float Odrive::readMotorControllerVoltage()
{
  float motor_controller_voltage;
  std::string command_name_ = this->create_command(O_PM_ODRIVE_INPUT_VOLTAGE);
  if (this->read(command_name_, motor_controller_voltage) == 1)
  {
    ROS_ERROR("Could not retrieve motor controller voltage");
    return ODRIVE_ERROR;
  }
  return motor_controller_voltage;
}

float Odrive::readMotorCurrent()
{
  float motor_current;
  std::string command_name_ = this->create_command(O_PM_ACTUAL_MOTOR_CURRENT);
  if (this->read(command_name_, motor_current) == 1)
  {
    ROS_ERROR("Could not retrieve motor current");
    return ODRIVE_ERROR;
  }
//  ROS_INFO("Motor_current: %f", motor_current);
  return motor_current;
}

float Odrive::readMotorVoltage()
{
  float motor_voltage;
  std::string command_name_ = this->create_command(O_PM_ACTUAL_MOTOR_VOLTAGE_D);
  if (this->read(command_name_, motor_voltage) == 1)
  {
    ROS_ERROR("Could not retrieve motor voltage");
    return ODRIVE_ERROR;
  }

  return motor_voltage;
}

int Odrive::readAngleCountsAbsolute()
{
  // There is no absolute encoder for the Odrive
  return 0;
}

double Odrive::readVelocityRadAbsolute()
{
  // There is no absolute encoder for the Odrive
  return 0;
}

double Odrive::readAngleCountsIncremental()
{
  int iu_position;
//  float iu_position;
  std::string command_name_ = this->create_command(O_PM_ENCODER_POSITION_UI);
  if (this->read(command_name_, iu_position) == 1)
  {
    ROS_ERROR("Could not retrieve incremental position of the encoder");
    return ODRIVE_ERROR
  }
  return iu_position;
}

double Odrive::readVelocityRadIncremental()
{
  float iu_velocity;
  std::string command_name_ = this->create_command(O_PM_ENCODER_VELOCITY_UI);
  if (this->read(command_name_, iu_velocity) == 1)
  {
    ROS_ERROR("Could not retrieve incremental velocity of the encoder");
    return ODRIVE_ERROR
  }

  double angle_rad = iu_velocity * PI_2 / GEAR_RATIO;
  return angle_rad;
}

std::string Odrive::create_command(std::string command_name)
{
  if (command_name.at(0) == '.')
  {
    return this->axis_number + command_name;
  }
  return command_name;
}


template int Odrive::validateType(const odrive_json_object& json_object, int8_t&);
template int Odrive::validateType(const odrive_json_object& json_object, int16_t&);
template int Odrive::validateType(const odrive_json_object& json_object, int32_t&);
template int Odrive::validateType(const odrive_json_object& json_object, int64_t&);
template int Odrive::validateType(const odrive_json_object& json_object, uint8_t&);
template int Odrive::validateType(const odrive_json_object& json_object, uint16_t&);
template int Odrive::validateType(const odrive_json_object& json_object, uint32_t&);
template int Odrive::validateType(const odrive_json_object& json_object, uint64_t&);
template int Odrive::validateType(const odrive_json_object& json_object, float&);
template int Odrive::validateType(const odrive_json_object& json_object, bool&);

template int Odrive::read(const std::string& parameter_name, int8_t&);
template int Odrive::read(const std::string& parameter_name, int16_t&);
template int Odrive::read(const std::string& parameter_name, int32_t&);
template int Odrive::read(const std::string& parameter_name, int64_t&);
template int Odrive::read(const std::string& parameter_name, uint8_t&);
template int Odrive::read(const std::string& parameter_name, uint16_t&);
template int Odrive::read(const std::string& parameter_name, uint32_t&);
template int Odrive::read(const std::string& parameter_name, uint64_t&);
template int Odrive::read(const std::string& parameter_name, float&);
template int Odrive::read(const std::string& parameter_name, bool&);

template int Odrive::write(const std::string& parameter_name, int8_t&);
template int Odrive::write(const std::string& parameter_name, int16_t&);
template int Odrive::write(const std::string& parameter_name, int32_t&);
template int Odrive::write(const std::string& parameter_name, int64_t&);
template int Odrive::write(const std::string& parameter_name, uint8_t&);
template int Odrive::write(const std::string& parameter_name, uint16_t&);
template int Odrive::write(const std::string& parameter_name, uint32_t&);
template int Odrive::write(const std::string& parameter_name, uint64_t&);
template int Odrive::write(const std::string& parameter_name, float&);
template int Odrive::write(const std::string& parameter_name, bool&);
}  // namespace march
