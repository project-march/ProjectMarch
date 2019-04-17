# xsense-awinda
ROS node for acquire xsense awinda device

This project is an integration of awinda monitor example provided by Xsense in ROS Framework
How to use:

1) Download and install MT Software Suite for Linux 
from :
https://www.xsens.com/mt-software-suite-mtw-awinda/

2) Clone this repository in your catkin_ws/src and compile it (catkin_make)
3) Modify config file "sensor_label.txt" in /cfg folder, adding your sensors' codes and labels

Possible problem: 
if lib is not found: 
  ibxsensdeviceapi.so.4: cannot open shared object file: No such file or directory
add path to your LD_LIBRARY_PATH
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/xsens/lib64/

if devices are not recognized, check that $USER is in dialout group

in ROS kinetic, if usr/include/boost/type_traits/detail/has_binary_operator.hp:50: Parse error at "BOOST_JOIN", refer to ROS answer: https://answers.ros.org/question/233786/parse-error-at-boost_join/
  The steps: 1. sudo gedit /usr/include/boost/type_traits/detail/has_binary_operator.hpp 2. Modify "namespace   BOOST_JOIN(BOOST_TT_TRAIT_NAME,_impl) {...}" to " #ifndef Q_MOC_RUN namespace BOOST_JOIN(BOOST_TT_TRAIT_NAME,_impl) { #endif .... #ifndef Q_MOC_RUN } #endif " 
  
