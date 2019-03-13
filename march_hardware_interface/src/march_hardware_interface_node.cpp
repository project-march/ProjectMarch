#include <march_hardware_interface/march_hardware_interface.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "march_hardware_interface");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    march_hardware_interface::MarchHardwareInterface march(nh);
    ros::spin();
    return 0;
}