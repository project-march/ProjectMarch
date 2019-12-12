// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_ETHERCAT_ETHERCATMASTER_H
#define MARCH_HARDWARE_ETHERCAT_ETHERCATMASTER_H
#include <vector>
#include <string>
#include <thread>  // NOLINT(build/c++11)

#include <march_hardware/Joint.h>

namespace march4cpp
{

/**
 * Base class of the ethercat master supported with the SOEM library
 * @param ifname Network interface name, check ifconfig.
 * @param IOmap Holds the mapping of the SOEM message.
 * @param expectedWKC The expected working counter of the ethercat train.
 * @param ecatCycleTimems The ethercat cycle time.
 * @param maxSlaveIndex The maximum amount of slaves connected to the train.
 */
    class EthercatMaster
    {
        std::string ifname;
        char IOmap[4096];
        int expectedWKC;

        std::thread EcatThread;
        std::vector<Joint>* jointListPtr;

        int maxSlaveIndex;
        int ecatCycleTimems;

    public:
        bool isOperational = false;

        explicit EthercatMaster(std::vector<Joint>* jointListPtr, std::string ifname, int maxSlaveIndex, int ecatCycleTime);
        ~EthercatMaster();

        void start();
        void ethercatMasterInitiation();
        void ethercatSlaveInitiation();

        void ethercatLoop();
        void SendReceivePDO();
        static void monitorSlaveConnection();

        void stop();
    };

}
#endif  // MARCH_HARDWARE_ETHERCAT_ETHERCATMASTER_H