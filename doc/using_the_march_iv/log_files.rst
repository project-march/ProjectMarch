Log files
=========

Rosbag
^^^^^^

The log files are recorded with a tool called rosbag.
Please check the `official documentation <http://wiki.ros.org/rosbag>`_ or :ref:`ros-bag-label`.


Exoskeleton
^^^^^^^^^^^
For each session, a new directory should be made with the following structure: **yyyy_mm_dd-activity_name**

Examples:
 - 2019-07-03_first_training
 - 2019-06-01_airgait_with_torque_mode


The files are located at: ``/home/march/Documents/log_files/<yyyy_mm_dd-activity_name>``

Cloudstation
^^^^^^^^^^^^

The log files are stored on the Cloudstation in a shared folder called **March IV log files**.
A special user **log** has been created to read/write these logs.


Uploading the logs
^^^^^^^^^^^^^^^^^^
First ssh into the computer of the exoskeleton.
The following command will upload all the logs to the cloudstation and create a folder with the correct <activity_name>.

.. todo:: @Isha, @Tim: Add link to ssh connection when it's done.

.. code::

  scp -r </path/to/yyyy_mm_dd-activity_name> log@cloudstation.projectmarch.nl:"/volume1/Log\ Files/March\ IV\ Log\ Files/"

