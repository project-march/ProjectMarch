
.. _connecting-to-the-exoskeleton-ethercat-label:

Connecting to the exoskeleton wired
===================================
.. inclusion-introduction-start

This tutorial will help you set up a wired connection with the exoskeleton.
If you are interested in connecting wirelessly, take a look at :ref:`connecting-to-the-exoskeleton-label`.

.. inclusion-introduction-end

.. note:: Make sure your workspace is sourced and set-up correctly: :ref:`setup-your-workspace-label`.

.. _preparation-before-connecting-ethercat-label:

Preparation before connecting wired
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Before you attempt to use a wired connection, you need to make sure :code:`ethercat_grant` is set up correctly.
:code:`ethercat_grant` is a package that is yet to be released for `Noetic <https://github.com/shadow-robot/ethercat_grant/issues/6>`_.
Until this package is released you will have to run a small install script manually.

Setting up ethercat_grant
--------------------------------------------------
We have written a small script that build the ethercat_grant package and sets up an executable at :code:`/usr/local/bin/`.
You can find the script here: :codedir:`setup_ethercat_grant.sh <using_the_march_exoskeleton/setup_ethercat_grant.sh>`

Run the script with:

.. code:: bash

    chmod +x setup_ethercat_grant.sh
    sudo bash -i -c "source /opt/ros/noetic/local_setup.bash && ./setup_ethercat_grant.sh"

If everything went well the :code:`ethercat_grant` executable should be in :code:`/usr/local/bin`.
You can verify this by running:

.. code:: bash

    ethercat_grant

The output should be exactly:

.. code:: bash

    terminate called after throwing an instance of 'std::logic_error'
      what():  basic_string::_M_construct null not valid
    Aborted (core dumped)

Setup configuration file
------------------------
After you have set up :code:`ethercat_grant`, make sure the robot .yaml file in :code:`ros1/src/hardware/interface/march_hardware_builder/robots/` you are using to launch ROS has the right :code:`ifName`.
You can set up the correct :code:`ifName` by running :code:`ifconfig` in a terminal and copying the name that starts with :code:`enp` to the .yaml file.

Launching ROS on the exoskeleton
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
After you performed both steps, you should rebuild the code. You should now be able to plug in an ethernet cable and launch ROS using the regular launch files!