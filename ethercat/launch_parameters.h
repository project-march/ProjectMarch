#ifndef LAUNCH_PARAMETERS_
#define LAUNCH_PARAMETERS_

#include <ros/ros.h>
#include <string>

using namespace std;

class LaunchParameters
{
public:
  static void init_parameters();

  static int* get_number_of_joints();
  static int* get_number_of_GES();
  static int* get_number_of_Pdb();
  static int* get_number_of_ipd();
  static int* get_ethercat_cycle_time();
  static int* get_ethercat_frequency();

  static vector<string>* get_list_of_joints();
  static vector<string>* get_list_of_GES();

  static int get_slave_number(string slaveName);
  static string get_slave_name(int stringNumber);

private:
  static int numberOfJoints, numberOfGES, numberOfPdb, numberOfIpd;
  static int ethercatCycleTime, ethercatFrequency;

  static vector<string> listOfJoints, listOfGES, listOfSlaves;

  static map<string, int> mapOfSlaveNamesNumbers;
};

#endif