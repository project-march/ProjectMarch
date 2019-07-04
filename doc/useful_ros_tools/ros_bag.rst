ROS bag
=======

.. note ::
  The official documentation for `rosbag <http://wiki.ros.org/rosbag>`_ and `rqt_bag <http://wiki.ros.org/rqt_bag>`_
  are quite comprehensive. Please refer to them if you have any questions.

`rosbag <http://wiki.ros.org/rosbag>`_ is an excellent tool which allows you to record any topics you want during runtime.
The recorded topics are saved to a bagfile. These bags can be easily inspected with an rqt_plugin called
`rqt_bag <http://wiki.ros.org/rqt_bag>`_ to plot the data.

.. figure:: images/example_ros_bag.png
   :align: center

   Example of rqt_bag

Start from launchfile
^^^^^^^^^^^^^^^^^^^^^

To record a rosbag when launching a launchfile, simply add the following node to your launchfile and change the topics.
Use the option -a to record all topics.

.. code::

  <node
    pkg="rosbag"
    name="record"
    type="record"
    output="screen"
    args="-o <output_directory> /topic1 /topic2"
  />
