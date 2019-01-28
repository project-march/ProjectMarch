#include "launch_parameters.h"

int LaunchParameters::numberOfJoints, 
	LaunchParameters::numberOfGES, 
	LaunchParameters::numberOfPdb, 
	LaunchParameters::numberOfIpd, 

	LaunchParameters::ethercatCycleTime, 
	LaunchParameters::ethercatFrequency;

vector<string> 
		LaunchParameters::listOfJoints, 
		LaunchParameters::listOfGES, 
		LaunchParameters::listOfSlaves;

map<string, int> LaunchParameters::mapOfSlaveNamesNumbers;

int* LaunchParameters::get_number_of_joints(){
	return &numberOfJoints;
}

int* LaunchParameters::get_number_of_GES(){
	return &numberOfGES;
}

int* LaunchParameters::get_number_of_Pdb(){
	return &numberOfPdb;
}

int* LaunchParameters::get_number_of_ipd(){
	return &numberOfIpd;
}

int* LaunchParameters::get_ethercat_cycle_time(){
	return &ethercatCycleTime;
}

int* LaunchParameters::get_ethercat_frequency(){
	return &ethercatFrequency;
}

vector<string>* LaunchParameters::get_list_of_joints(){
	return &listOfJoints;
}

vector<string>* LaunchParameters::get_list_of_GES(){
	return &listOfGES;
}

string LaunchParameters::get_slave_name(int slaveNumber){
	return listOfSlaves[slaveNumber];
}

int LaunchParameters::get_slave_number(string slaveName)
{
	map<string, int>::iterator it = mapOfSlaveNamesNumbers.find(slaveName);

	if(it == mapOfSlaveNamesNumbers.end())
		return -1;

	return it->second;
}

//-----------------------------------------------------------------------------------------------//

void LaunchParameters::init_parameters(){

	bool success = true;

	success &= ros::param::get("/NUMBER_OF_JOINTS", numberOfJoints);
	success &= ros::param::get("/NUMBER_OF_GES", numberOfGES);
	success &= ros::param::get("/NUMBER_OF_PDB", numberOfPdb);
	success &= ros::param::get("/NUMBER_OF_IPD", numberOfIpd);

	success &= ros::param::get("/ETHERCAT_CYCLE_TIME", ethercatCycleTime);
	ethercatFrequency = 1000/ethercatCycleTime;

	success &= ros::param::get("/LIST_OF_JOINTS", listOfJoints);
	success &= ros::param::get("/LIST_OF_GES", listOfGES);
	
	success &= ros::param::get("/LIST_OF_SLAVES", listOfSlaves);
	for (int i = 0; i < listOfSlaves.size(); i++)
	{
		mapOfSlaveNamesNumbers[listOfSlaves[i]] = i;		
	}

	cout << "the param getting was: ";
	if (success){
		cout << "a success! :D" << endl;
	}
	else{
		cout << "a failure! D:<" << endl;
	}

}