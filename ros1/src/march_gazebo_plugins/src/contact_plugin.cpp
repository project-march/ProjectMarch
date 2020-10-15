#include <contact/contact_plugin.h>

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

/////////////////////////////////////////////////
ContactPlugin::ContactPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
ContactPlugin::~ContactPlugin()
{
}

/////////////////////////////////////////////////
void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  // Get the parent sensor.
  this->parentSensor = std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ContactPlugin requires a ContactSensor.\n";
    return;
  }
  this->name = this->parentSensor->Name();

  // Create our ROS node.
  this->ros_node_ = std::make_unique<ros::NodeHandle>(this->name);

  // Create a named topic, and subscribe to it.
  std::ostringstream topic_name;
  topic_name << "/march/contact/" << this->name;
  this->ros_pub_ = this->ros_node_->advertise<std_msgs::Bool>(topic_name.str(), 10);

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(std::bind(&ContactPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);

  // Log the initialization
  std::ostringstream initialized;
  initialized << "Succesfully initialized contact plugin: " << this->name;
  ROS_INFO_STREAM(initialized.str());
}

/////////////////////////////////////////////////
void ContactPlugin::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
  std_msgs::Bool msg;
  msg.data = false;
  if (contacts.contact_size() > 0)
  {
    msg.data = true;
  }
  this->ros_pub_.publish(msg);
}