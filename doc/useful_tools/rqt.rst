.. _rqt-label:

RQT
===
`RQT <https://wiki.ros.org/rqt>`_ is a Qt-based framework for GUI development in ROS.
All GUI applications from ROS are written using RQT, for example, ``rviz`` and ``ros_bag``.
The GUI applications are written as RQT plugins, which can be included in windows, so one
window can include multiple RQT plugins. For example, if you run

.. code::

    rqt  # a roscore has to be running

You can select multiple plugins from the ``Plugins`` menu in the top left. Once you have
selected a few plugins, you can save the current 'perspective'. A perspective is a combination
of RQT plugins in a defined configuration. In the ``Perspectives`` menu you can save, import,
export and set perspectives as default. This is useful if you want, for example, to have the
input device and the gait selection tool next to each other in one window.

.. figure:: images/rqt_perspective.png
   :align: center

   Example rqt perspective

The :monitor:`project-march/monitor <>` repository contains RQT plugins used
to monitor and control the |march| and the :gait-generation:`gait generator <>` is also an RQT plugin.

See also
^^^^^^^^
* :ref:`how-to-view-live-data-label`
* :ref:`using-the-gait-generator-label`
