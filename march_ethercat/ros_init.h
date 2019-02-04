#ifndef ROS_INIT_
#define ROS_INIT_

#include <ros/ros.h>
#include <string>
#include "imc_callback.h"
#include "ipd_callback.h"
#include "update.h"

// import message types
#include <custom_msgs/ECtoJH.h>
#include <custom_msgs/ECtoUG.h>
#include <custom_msgs/ECtoLG.h>
#include <custom_msgs/ECtoPDB.h>
#include <custom_msgs/ECtoIPD.h>

#include <custom_msgs/data8Msg.h>

#include "launch_parameters.h"

// JOINTHANDLER
static vector<ros::Subscriber*> jointPositionSubList;
static vector<ros::Subscriber*> jointWordSubList;
static map<string, ros::Publisher*> jointHandlerPubMap;

// GESHANDLER
static map<string, ros::Publisher*> gesHandlerPubMap;
static ros::Subscriber* speakerSubPtr;

// SAFETY
static ros::Publisher* safetyPubPtr;

// PDBHANDLER
static ros::Publisher* PdbHandlerPubPtr;
static ros::Subscriber* PdbCommandSubPtr;

// IPDHANDLER
static ros::Publisher* IpdHandlerPubPtr;

static ros::Subscriber* IpdHandlerSubPtr;

// TEMPLATE GES HANDLER
static ros::Subscriber* TemplateHandlerSubPtr;

void rosloop(ros::NodeHandle nh);

void publish_to_joint_handler(string slaveName, custom_msgs::ECtoJH msg);

void publish_to_ges_handler(string slaveName, custom_msgs::ECtoUG msg);
void publish_to_ges_handler(string slaveName, custom_msgs::ECtoLG msg);
// void publish_to_ges_handler(string slaveName, custom_msgs::ECtoBPG msg);
void publish_to_pdb_handler(custom_msgs::ECtoPDB msg);
void publish_to_ipd_handler(custom_msgs::ECtoIPD msg);

void publish_to_safety(custom_msgs::data8Msg msg);

void publish_to_ipd_handler(string slaveName, custom_msgs::ECtoIPD msg);

void rosSpinOnce();

#endif  // ROS_INIT_