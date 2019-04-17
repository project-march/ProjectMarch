/**
 * @file /include/qtcontroller/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef awindamonitor_QNODE_HPP_
#define awindamonitor_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QString>
#include <QMessageBox>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <map>


/*****************************************************************************
** Namespaces
*****************************************************************************/

//namespace awindamonitor {

/*****************************************************************************
** Class
*****************************************************************************/
typedef std::map<std::string, ros::Publisher> MapPublishers;



class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();
    void showNoMasterMessage();
    ros::Publisher acc_test_publisher;
    MapPublishers publishers; 
    MapPublishers publishers_rpy; 
    
    //void postDcartCommand(QString);
    //void postCoroCommand(QString);

    //QString getDcartStatus();
    //QString getCoroStatus();

Q_SIGNALS:
    void rosShutdown();
    //void dcartStatusChanged();
    //void coroStatusChanged();

private:
	int init_argc;
	char** init_argv;
	//ros::Publisher coro_command_publisher;
	//ros::Subscriber dcart_status_subscriber;
	//ros::Subscriber coro_status_subscriber;
    //void callbackDcartStatus(std_msgs::String);
    //void callbackCoroStatus(std_msgs::String);

    //QString dcart_status;
    //QString coro_status;
};

//}  // namespace awindamonitor 

#endif /* qtcontroller_QNODE_HPP_ */
