.. _smach-label:

SMACH
=====

.. note ::
  The official documentation for `smach <http://wiki.ros.org/smach>`_ is quite comprehensive. Please refer to it if you have any questions.

Introduction
^^^^^^^^^^^^
SMACH is a task-level architecture for rapidly creating complex robot behavior.
At its core, SMACH is a ROS-independent Python library to build hierarchical state machines.

SMACH makes it easy to get an overview of the behavior of the exoskeleton.
It includes a visualization called the Introspection Server.

SMACH is well documented on the `ros wiki <http://wiki.ros.org/smach>`_.
If you have to work with SMACH, make sure you have completed all their tutorials first.

.. figure:: images/smach.png
   :align: center

   Snippet of the March state machine

smach_viewer
^^^^^^^^^^^^
SMACH also contains a viewer. This displays all state and transitions of the state machine. This visualization can highlight the current state during runtime.
This package is documented `here <http://wiki.ros.org/smach_viewer>`_.

.. tip::
  To get started read a few `SMACH tutorials or examples <http://wiki.ros.org/smach/Tutorials>`_.
