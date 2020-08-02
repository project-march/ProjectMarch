.. _march-ems-projects-label:

march_ems_projects
==================

Overview
--------
The march_ems_projects package contains the EasyMotion Studio (ems) projects of the iMotionCubes. The other folders in the
ems-projects repository contain old projects which you will hopefully never need. march_ems_projects/ems_projects
contains the projects you can open in EasyMotionStudio. The sw_files folder contains the corresponding sw_files; this
is the format in which the project is saved on the iMC. At startup, we check whether the sw file on the iMC still
Matches that on the master, and if not, we rewrite the sw file via ethercat. See IMotionCube::verifySetup in
:hardware-interface:`imotioncube.cpp </march_hardware/src/imotioncube/imotioncube.cpp>`.

Creating an sw-file
-------------------

When you change one of the ems projects, the corresponding sw file must also be updated. You can do this via
"file/export to EEPROM/setup only", see image below.

.. figure:: images/making_an_sw_file.png
   :align: center

   How to make an sw file
