#include "ros_init.h"

extern "C" {
#include "osal.h"
}

//------------------------------------------------------------------------------------------------//
// JOINT HANDLER PUB FUNCTIONS
//------------------------------------------------------------------------------------------------//

void publish_to_joint_handler(string slaveName, custom_msgs::ECtoJH msg)
{
  (jointHandlerPubMap.find(slaveName)->second)->publish(msg);
}

//------------------------------------------------------------------------------------------------//

//------------------------------------------------------------------------------------------------//
// GES HANDLER PUB FUNCTIONS
//------------------------------------------------------------------------------------------------//

void publish_to_ges_handler(string slaveName, custom_msgs::ECtoUG msg)
{
  (gesHandlerPubMap.find(slaveName)->second)->publish(msg);
}

void publish_to_ges_handler(string slaveName, custom_msgs::ECtoLG msg)
{
  (gesHandlerPubMap.find(slaveName)->second)->publish(msg);
}

// void publish_to_ges_handler(string slaveName, custom_msgs::ECtoBPG msg)
// {
// 	(gesHandlerPubMap.find(slaveName)->second)->publish(msg);
// }

//------------------------------------------------------------------------------------------------//

//------------------------------------------------------------------------------------------------//
// GES HANDLER PUB FUNCTIONS
//------------------------------------------------------------------------------------------------//

void publish_to_safety(custom_msgs::data8Msg msg)
{
  safetyPubPtr->publish(msg);
}

//------------------------------------------------------------------------------------------------//

//------------------------------------------------------------------------------------------------//
// GES HANDLER PUB FUNCTIONS
//------------------------------------------------------------------------------------------------//

void publish_to_pdb_handler(custom_msgs::ECtoPDB msg)
{
  PdbHandlerPubPtr->publish(msg);
}

void publish_to_ipd_handler(custom_msgs::ECtoIPD msg)
{
  IpdHandlerPubPtr->publish(msg);
}

// ros spinOnce call
void rosSpinOnce()
{
  // pulls from subscribed topics and executes callback functions
  ros::spinOnce();
}

void rosloop(ros::NodeHandle nh)
{
  //------------------------------------------------------------------------------------------------//
  // JOINT HANDLER PUBS & SUBS
  //--//--------------------------------------------------------------------------------------------//
  // 	UPSTREAM COMBINED MESSAGES
  //--------------------------------------------------------------------------------------------//

  for (int i = 0; i < *LaunchParameters::get_number_of_joints(); i++)
  {
    // create publishers w/ datatype, topic name and queue size
    string prefix = (*LaunchParameters::get_list_of_joints())[i];

    jointHandlerPubMap[prefix] = (new ros::Publisher(nh.advertise<custom_msgs::ECtoJH>("ECto" + prefix, 50)));

    //--------------------------------------------------------------------------------------------//
    // TARGET POSITION
    //--------------------------------------------------------------------------------------------//

    // create subscribers w/ topic name, queue size and callback function
    jointPositionSubList.push_back(
        new ros::Subscriber(nh.subscribe(prefix + "TargetPositionIU", 1, &Imc_callback::set_target_position)));

    //--------------------------------------------------------------------------------------------//
    // CONTROLWORD
    //--------------------------------------------------------------------------------------------//

    // create subscribers w/ topic name, queue size and callback function
    jointWordSubList.push_back(
        new ros::Subscriber(nh.subscribe(prefix + "Controlword", 50, &Imc_callback::set_controlword)));
  };

  //------------------------------------------------------------------------------------------------//

  //------------------------------------------------------------------------------------------------//
  // GES HANDLER PUBS & SUBS
  //--//--------------------------------------------------------------------------------------------//
  // COMBINED MESSAGES UPSTREAM
  //--------------------------------------------------------------------------------------------//

  // for(int i = 0; i < *get_number_of_GES(); i++)
  // {
  // 	//create publishers w/ datatype, topic name and queue size
  // 	string prefix = (*get_list_of_GES())[i];

  // 	gesHandlerPubMap[prefix] = (
  // 		new ros::Publisher(
  // 			nh.advertise<custom_msgs::ECtoUG>(
  // 				"ECto" + prefix,
  // 				50
  // 			)
  // 		)
  // 	);

  // };

  for (vector<string>::iterator i = LaunchParameters::get_list_of_GES()->begin();
       i != LaunchParameters::get_list_of_GES()->end(); ++i)
  {
    if (*i == "LUG" || *i == "RUG")
    {
      gesHandlerPubMap[*i] = (new ros::Publisher(nh.advertise<custom_msgs::ECtoUG>("ECto" + *i, 50)));
    }
    else if (*i == "LLG" || *i == "RLG")
    {
      gesHandlerPubMap[*i] = (new ros::Publisher(nh.advertise<custom_msgs::ECtoLG>("ECto" + *i, 50)));
    }
    else if (*i == "BPG")
    {
      // gesHandlerPubMap[*i] = (
      // 	new ros::Publisher(
      // 		nh.advertise<custom_msgs::ECtoBPG>(
      // 			"ECto" + *i,
      // 			50
      // 		)
      // 	)
      // );

      speakerSubPtr = (new ros::Subscriber(nh.subscribe(*i + "toEC", 50, &Backpack_ges_callback::send_frequency)));
    }
    else
    {
      printf("error when making ges publishers! unknown name\n");
    }
  }

  //------------------------------------------------------------------------------------------------//

  //------------------------------------------------------------------------------------------------//
  // SAFETY PUBS & SUBS
  //--//--------------------------------------------------------------------------------------------//
  // SLAVE LOST ERROR
  //--------------------------------------------------------------------------------------------//

  safetyPubPtr = (new ros::Publisher(nh.advertise<custom_msgs::data8Msg>("ECErrorMessage", 50)));

  //------------------------------------------------------------------------------------------------//

  //------------------------------------------------------------------------------------------------//
  // PDB HANDLER PUBS & SUBS
  //--//--------------------------------------------------------------------------------------------//
  // UPSTREAM COMBINED MESSAGES
  //--------------------------------------------------------------------------------------------//
  if (*LaunchParameters::get_number_of_Pdb())
  {
    PdbHandlerPubPtr = (new ros::Publisher(nh.advertise<custom_msgs::ECtoPDB>("ECtoPDB", 50)));

    //--------------------------------------------------------------------------------------------//
    // PDB command
    //--------------------------------------------------------------------------------------------//

    PdbCommandSubPtr = (new ros::Subscriber(nh.subscribe("PDBtoEC", 50, &Pdb_callback::set_pdb_command)));
  }

  //------------------------------------------------------------------------------------------------//
  // IPD HANDLER PUBS & SUBS
  //--//--------------------------------------------------------------------------------------------//
  // UPSTREAM COMBINED MESSAGES
  //--------------------------------------------------------------------------------------------//

  if (*LaunchParameters::get_number_of_ipd())
  {
    IpdHandlerPubPtr = (new ros::Publisher(nh.advertise<custom_msgs::ECtoIPD>("ECtoIPD", 50)));

    //--------------------------------------------------------------------------------------------//
    // IPD DOWN
    //--------------------------------------------------------------------------------------------//

    IpdHandlerSubPtr = (new ros::Subscriber(nh.subscribe("IPDtoEC", 50, &Ipd_callback::set_ipd_data)));
  }
  //------------------------------------------------------------------------------------------------//

  cout << "number of joints: " << *LaunchParameters::get_number_of_joints() << endl;
  cout << "number of ges: " << *LaunchParameters::get_number_of_GES() << endl;
  cout << "number of pdb: " << *LaunchParameters::get_number_of_Pdb() << endl;
  cout << "number of ipd: " << *LaunchParameters::get_number_of_ipd() << endl;

  // startup_sdo(1);

  ros::Rate rate(*LaunchParameters::get_ethercat_frequency());
  while (ros::ok())
  {
    update();

    ros::spinOnce();

    rate.sleep();
  };

  cout << "escape!" << endl;
}