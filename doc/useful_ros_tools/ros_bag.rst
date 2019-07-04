.. _ros-bag-label:

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

Recording bags
^^^^^^^^^^^^^^

There are many ways to record a rosbag: Using the RQT plugin, with the commandline, or adding it to the launchfile.

Adding it to the launchfile is the least effort, and is done by starting the following node:

.. code::

  <node
    pkg="rosbag"
    name="record"
    type="record"
    output="screen"
    args="-o <output_directory> /topic1 /topic2"
  />

Use the option -a to record all topics.