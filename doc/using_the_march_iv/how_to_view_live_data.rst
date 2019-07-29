How to view live data
=====================

Introduction
^^^^^^^^^^^^
This tutorial will teach you how to use the `rqt_multiplot <http://wiki.ros.org/rqt_multiplot>`_ tool to visualize ros data during run-time

rqt_multiplot
^^^^^^^^^^^^^
`rqt_multiplot <http://wiki.ros.org/rqt_multiplot>`_ is an advanced version of `rqt_plot <http://wiki.ros.org/rqt_plot>`_
which allows the user to easily create many plots with different topics.

Please see the documentation linked above as it covers everything you need to know to use it.

.. figure:: images/rqt_multiplot.png
   :align: center

   Example rqt_multiplot configuration

Configuration
^^^^^^^^^^^^^
rqt_multiplot allows you to save a configuration that determines which plots are displayed in the ui.
As configuring a configuration can be time consuming, we have created a :march-iv_repo:`configuration folder<march_launch/rqt_multiplot>>` with commonly used configurations to easily visualize the data you want.

If you have created an additional useful configuration, feel free to open a pull request to have it added to the folder.

You can save and load configurations in the top right of the application.