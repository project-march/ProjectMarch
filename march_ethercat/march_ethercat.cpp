/*
 * march_ethercat       - Initialises and runs SOEM/Ethercat and the ros node
 *
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <ros/ros.h>

extern "C" {
#include "ethercat.h"
#include "osal.h"
}

#include "ros_init.h"
#include "update.h"

#define EC_TIMEOUTMON 500

char IOmap[4096];               // holds the mapping of the SOEM message
OSAL_THREAD_HANDLE thread1;     // holds the thread used for error handling with soem
int expectedWKC;                // expected working-counter
boolean needlf;                 // ???? Doing nothing useful right now
volatile int wkc;               // keeps track of SOEMs working-counter, can be used as a safety check toghther with the expected wkc to see if every slave is passed
boolean inOP;                   // Whether or not SOEM is currently in Operational state
uint8 currentgroup = 0;         // Identifier for the current slave group

void march_ethercat(int argc, char* argv[])
{

  // --------------------------------------------------------------------------------
  // initialise rosnode
  ros::init(argc, argv, "ethercat_node");
  // create nodeHandle object which represents the current ros node
  ros::NodeHandle nh;

  LaunchParameters::init_parameters();

  // --------------------------------------------------------------------------------

  char* ifname = argv[1];       // Holds the name of the network interface (if) over which SOEM will be run
  int i, chk;  //, j, oloop, iloop;
  needlf = FALSE;
  inOP = FALSE;

  printf("Starting ethercat\n");

  /* initialise SOEM, bind socket to ifname */
  if (ec_init(ifname))
  {
    printf("ec_init on %s succeeded.\n", ifname);
    /* find and auto-config slaves */

    if (ec_config_init(FALSE) > 0)
    {
      printf("%d slaves found and configured.\n", ec_slavecount);

      // request and wait for slaves to be in pre-operational state (currently points to slave 1 which puzzles me, id expect this to point to 0 (all slaves))
      ec_statecheck(1, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 4);     


      // ----------------------------------------------------------------------------

      // over all iMotioncubes: 
      //    apply PDO-mapping                 --> This can only be done while in pre-operational mode (if i remember correctly this is defined in the iMc documentation)
      //    send initialisation SDO messages

      for (int j = 0; j < *LaunchParameters::get_number_of_joints(); j++)
      {
        int slaveNumber = LaunchParameters::get_slave_number((*LaunchParameters::get_list_of_joints())[j]);
        pdo_mapping(slaveNumber);
        startup_sdo(slaveNumber);
      }

      // ----------------------------------------------------------------------------

      // Configure the ethercat message structure depending on the PDO mapping of all the slaves
      ec_config_map(&IOmap);

      ec_configdc();

      /* wait for all slaves to reach SAFE_OP state */
      ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);


      // Safety check on the data size fo the input and output of the slaves (afaik?)
      /*
       oloop = ec_slave[0].Obytes;
       if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
       if (oloop > 8) oloop = 8;

       iloop = ec_slave[0].Ibytes;
       if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
       if (iloop > 8) iloop = 8;
      */

      printf("segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1],
             ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

      printf("Request operational state for all slaves\n");
      expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
      printf("Calculated workcounter %d\n", expectedWKC);
      ec_slave[0].state = EC_STATE_OPERATIONAL;

      /* send one valid process data to make outputs in slaves happy*/
      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);

      /* request OP state for all slaves */
      ec_writestate(0);
      chk = 40;

      /* wait for all slaves to reach OP state */
      do
      {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
      } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));


      // All slaves in operational
      if (ec_slave[0].state == EC_STATE_OPERATIONAL)
      {
        // printf("Operational state reached for all slaves.\n");
        printf("Slave name: %s\n", ec_slave[0].name);
        inOP = TRUE;

        // Send and receive ethercat message
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);

        // Run ros node program
        rosloop(nh);

        inOP = FALSE;
      }
      else
      {
        printf("Not all slaves reached operational state.\n");
        ec_readstate();
        for (i = 1; i <= ec_slavecount; i++)
        {
          if (ec_slave[i].state != EC_STATE_OPERATIONAL)
          {
            printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n", i, ec_slave[i].state, ec_slave[i].ALstatuscode,
                   ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
          }
        }
      }
      printf("\nRequest init state for all slaves\n");
      ec_slave[0].state = EC_STATE_INIT;
      /* request INIT state for all slaves */
      ec_writestate(0);
    }
    else
    {
      printf("No slaves found!\n");
    }
    printf("End simple test, close socket\n");
    /* stop SOEM, close socket */
    ec_close();
  }
  else
  {
    printf("No socket connection on %s\nExcecute as root\n", ifname);
  }
}



// Error handling function for SOEM. Is run in a seperate thread next to SOEM itself. 
OSAL_THREAD_FUNC ecatcheck(void* ptr)
{
  int slave;
  (void)ptr; /* Not used */

  while (1)
  {
    if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
    {
      if (needlf)
      {
        needlf = FALSE;
        printf("\n");
      }
      /* one ore more slaves are not responding */
      ec_group[currentgroup].docheckstate = FALSE;
      ec_readstate();
      for (slave = 1; slave <= ec_slavecount; slave++)
      {
        if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
        {
          ec_group[currentgroup].docheckstate = TRUE;
          if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
          {
            printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
            ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
            ec_writestate(slave);
          }
          else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
          {
            printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
            ec_slave[slave].state = EC_STATE_OPERATIONAL;
            ec_writestate(slave);
          }
          else if (ec_slave[slave].state > EC_STATE_NONE)
          {
            if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
            {
              ec_slave[slave].islost = FALSE;
              printf("MESSAGE : slave %d reconfigured\n", slave);
            }
          }
          else if (!ec_slave[slave].islost)
          {
            /* re-check state */
            ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
            if (ec_slave[slave].state == EC_STATE_NONE)
            {
              ec_slave[slave].islost = TRUE;
              printf("ERROR : slave %d lost\n", slave);
            }
          }
        }
        if (ec_slave[slave].islost)
        {
          if (ec_slave[slave].state == EC_STATE_NONE)
          {
            if (ec_recover_slave(slave, EC_TIMEOUTMON))
            {
              ec_slave[slave].islost = FALSE;
              printf("MESSAGE : slave %d recovered\n", slave);
            }
          }
          else
          {
            ec_slave[slave].islost = FALSE;
            printf("MESSAGE : slave %d found\n", slave);
          }
        }
      }
      // if(!ec_group[currentgroup].docheckstate)
      //    printf("OK : all slaves resumed OPERATIONAL.\n"); // removed to reduce prints
    }
    osal_usleep(10000);
  }
}

int main(int argc, char* argv[])
{

  // Runs the program if the right amount of variables have been given
  if (argc >= 2)
  {
    // create thread to handle slave error handling in OP (operational mode)
    march_osal_thread_create(&thread1, 128000, &ecatcheck, &ctime);

    // initialise the ROS node and SOEM
    march_ethercat(argc, argv);
  }
  else
  {

    // probably outdated advise for the usage of led_test without launch file 
    printf("Usage: roslaunch march_ethercat march_ethercat.launch ifname1\nif unsure use ifconfig in terminal\n");
  }

  printf("End of march_ethercat\n");
  return (0);
}
