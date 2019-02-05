//
// Created by Martijn on 4-2-19.
//

#ifndef PROJECT_ETHERCAT_H
#define PROJECT_ETHERCAT_H

#include <ros/node_handle.h>
#include "ethercat_io.h"
#include "Slave.h"

extern "C" {
#include "osal.h"
}

class EthercatMaster {
    std::string ifname;     // Network interface name, check ifconfig
    char IOmap[4096];       // Holds the mapping of the SOEM message
    int expectedWKC;        // Expected working counter

public:
    bool inOP;           // Is SOEM in operational state

    // Constructor
    explicit EthercatMaster(ros::NodeHandle nh, std::vector<Slave> slave);
    // Destructor
    ~EthercatMaster();
    // Send Process Data over EtherCAT
    void SendProcessData();
    // Receive Process Data over EtherCAT
    int ReceiveProcessData();
    // Call Callbacks for received Process Data
    void PublishProcessData();
    // Monitor slave connection
    void MonitorSlaveConnection();
    // Set data to be sent
    void SetByte(std::string slavename, uint8 offset, int8 byte);
    // Get input received
    int8 GetByte(std::string slavename, uint8 offset);
    // PDO mapping
    int PDOmapping(int slave);
    // Startup SDO
    int StartupSDO(int slave);

};

#endif //PROJECT_ETHERCAT_H
