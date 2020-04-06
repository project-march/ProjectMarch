.. _error-codes:

Error Codes
===========
This page tries to document all possible errors that can arise.
Every error is documented with causes and possible fixes.
Error codes can be recognized by and ``E`` followed by three digits.
Error codes start counting at ``100`` and for now there are is no structure in how the errors are sorted.
This way error codes can easily be recognized and communicated with others,
without having to repeat the whole error description.

.. note:: This page is still a work in progress and might not contain all errors you can encounter.
          If you encounter an error that is not documented, consider creating
          `an issue <https://github.com/project-march/tutorials/issues/new/choose>`_ or
          `pull request <https://github.com/project-march/tutorials>`_.

**Error list**

.. contents:: :local:


..  From here start the error descriptions. Every error is formatted as a
    subsection that starts with '``EXXX``: ' followed by a short title of the error.
    Furthermore, every error subsection should be preceded by a label of the error, i.e. '.. _exxx:'.
    Finally, the error subsection should contain a short description, causes and possible fixes.


.. _e100:

``E100``: Invalid actuation mode
--------------------------------
An invalid actuation mode was used to perform an action.

**Cause:** The motor controllers have two modes for actuation: position and effort.
With position, you can only actuate to a target position using ``actuateRad()``.
In effort mode you can only actuate using a target torque using ``actuateTorque()``.
This error was caused by mixing the actuate methods and actuation modes. See
the detailed description for which mode the controller was actually in.

**Fix:** Check if the actuation mode defined in the :hardware-interface:`robots yaml <march_hardware_builder/robots>`
you use is consistent with the methods you are using to actuate the joint.


.. _e101:

``E101``: Invalid actuate position
----------------------------------
Invalid IU position command.

**Cause:** The position you are trying to actuate to is not a valid target within the range of the joint.

**Fix:** Check that the IU limits defined in the :hardware-interface:`robots yaml <march_hardware_builder/robots>`
under ``joints/<joint_name>/imotioncube/encoder`` match the actual limits of the joint.


.. _e102:

``E102``: Encoder reset
-----------------------
An encoder has reset and reads an incorrect value.

**Cause:** The cause is still unknown.

**Fix:** This error sometimes happens during startup and the only thing you
can do is try again. This error will probably be removed once this issue has
been fixed.


.. _e103:

``E103``: Outside hard limits
-----------------------------
A joint is outside its defined hard limits.

**Cause:** A joint was found to be outside its defined hard limits in the
:hardware-interface:`robots yaml <march_hardware_builder/robots>`.

**Fix:** Check that the IU limits defined in the :hardware-interface:`robots yaml <march_hardware_builder/robots>`
under ``joints/<joint_name>/imotioncube/encoder`` match the actual limits of the joint and recalibrate if necessary.


.. _e104:

``E104``: Target exceeds max difference
---------------------------------------
The target position exceeds the max allowed difference from the current position.

**Cause:** The given target position is greater than the maximum allowed difference
between target and actual position. This could cause dangerous situations if
you were to actuate a long distance in a short amount of time.

**Fix:** This is a safety measure and most of the time indicates that the joints
are behind their trajectory path.


.. _e105:

``E105``: Target torque exceeds max torque
------------------------------------------
The target torque exceeds the max allowed torque

**Cause:** The maximum given target torque is greater than the joints
can actuate. This could cause dangerous situations where the joints could be
negatively affected.

**Fix:** This is a safety measure and most of the time indicates that the joints
are behind their trajectory path.


.. _e106:

``E106``: PDO object not defined
--------------------------------
The to be added PDO object was not defined.

**Cause:** The PDO that is being mapped does not have a defined address and size.

**Fix:** Check that the PDO is defined in the ``all_objects`` map in the
:hardware-interface:`PDOmap <march_hardware/src/PDOmap.cpp>` class.


.. _e107:

``E107``: PDO register overflow
-------------------------------
The PDO map could not fit within the registers

**Cause:** There exist 4 registers with 64 bits each to be filled with PDO maps.
This error indicates that the added PDO exceeded 4*64=256 bits.

**Fix:** Check that you actually need all the mapped PDOs and remove some that
you are not using until you are at or below the 256 bits.


.. _e108:

``E108``: Writing initial settings failed
-----------------------------------------
Failed to write initial settings to slave required for operation.

**Cause:** Before setting all ethercat slaves to operational mode, the master
writes some settings required during operation. If such a write command fails,
it means that the master did not get any confirmation that the value was written.
This could have several causes: A slave was (temporarily) lost during writing
or the slave does not allow writing the value to that address.

**Fix:** Check the connection between the faulty slave and the master or
check that you write the correct sized (8, 16, 32 bit) value to the correct address.


.. _e109:

``E109``: No socket connection
------------------------------
The ethercat master failed to open a raw network socket.

**Cause:** This can have several causes:

1. The master tries to open a socket with an ``ifname`` that does not exist.
2. The user executing the program does not have permissions for opening a raw socket. Only root can do this.
3. The ethernet port on the machine is not connected.

**Fix:** First, make sure that you connected your machine to a slave.
Next, check if the ``ifname`` defined in the :hardware-interface:`robots yaml <march_hardware_builder/robots>`
has an existing socket name. You can list available sockets with

.. code::

  ip link show

The permissions for opening raw sockets while executing the hardware interface
are added by `ethercat_grant <https://github.com/shadow-robot/ethercat_grant>`_.
So this should not be an issue unless you changed something very specific.


.. _e110:

``E110``: Not all slaves found
------------------------------
The ethercat master was not able to find all configured slaves during initialization.

**Cause:** The ethercat master was not able to establish a connection with the configured amount of slaves
in the :hardware-interface:`robots yaml <march_hardware_builder/robots>`, because a cable is not connected or the
configuration contains more slaves than actually connected.

**Fix:** Check if all ethernet cables are correctly connected to the in- and outputs of the slaves you want to
connect to. Finally, check if the slaves configured in the :hardware-interface:`robots yaml <march_hardware_builder/robots>`
have the correct slave indices.


.. _e111:

``E111``: Failed to reach operational state
-------------------------------------------
At least one slave was not able to reach operational state.

**Cause:** The connection to the slave was lost during initialization or the PDO mapping is incorrect.

**Fix:** The error lists the slaves that were not able to go to operational state,
so check the connection on those specific slaves. If you made any changes to the PDO mapping,
verify that those are correct.


.. _e112:

``E112``: Invalid encoder resolution
------------------------------------
The encoder resolution is outside the allowed range.

**Cause:** The given encoder resolution to construct an encoder are outside its allowed limits defined in
:hardware-interface:`Encoder.h <march_hardware/include/march_hardware/encoder/Encoder.h>`.

**Fix:** Check if the resolutions given in the :hardware-interface:`robots yaml <march_hardware_builder/robots>` are
within this range.


.. _e114:

``E114``: Invalid range of motion
---------------------------------
The lower and upper limits of an encoder are conflicting.

**Cause:** When an encoder is constructed it checks that its lower limits are below its upper limits, for hard and soft
limits. Furthermore, it checks if the soft limits are within the defined hard limits. This is to make sure that the
joints can safely actuate.

**Fix:** The encoder limits are defined in the :hardware-interface:`robots yaml <march_hardware_builder/robots>`, so
make sure that the lower limit is lower than the upper limit in incremental units. If the encoder is built from the
``march_hardware_builder`` then the soft limits are extracted from the :march:`URDF <march_description/urdf>` of the used robot.
Make sure that the limits in the URDF are also non-conflicting.


.. _e115:

``E115``: Invalid slave configuration
-------------------------------------
The slave configuration contains duplicate slave indices

**Cause:** Before the ethercat train is started, the configured slaves indices are checked for duplicate indices.
This makes sure that no two controllers will write to the same slave.

**Fix:** Check the :hardware-interface:`robots yaml <march_hardware_builder/robots>`, so that it does not contain
duplicate indices on iMOTIONCUBES. Temperature sensors can have duplicate slave indices, since multiple temperature
sensors can be connected to a slave.


.. _e116:

``E116``: Not allowed to actuate
--------------------------------
A joint is not allowed to actuate, yet it's trying to actuate.

**Cause:** A joint must have enabled actuation in order to actuate. This value can be set in the
:hardware-interface:`robots yaml <march_hardware_builder/robots>`.

**Fix:** Check that the joints that you are trying to actuate are actually allowed to actuate.


.. _e117:

``E117``: Invalid slave index
-----------------------------
Slave index has an invalid value.

**Cause:** When slaves are created with a slave index, it is made sure that the indices are not lower than 1.
Since index 0 is the master itself and values lower than 0 are not valid indices.

**Fix:** If you are using a :hardware-interface:`robots yaml <march_hardware_builder/robots>`, make sure that all
slave indices are defined as integers higher than 0.


.. _e118:

``E118``: Missing URDF joint
----------------------------
A required joint was not defined in URDF.

**Cause:** The ``march_hardware_builder`` package uses the URDF and the robots
yaml for building a March robot. The joints defined in the URDF are required for
soft and hard limits of the joint and the builder cannot continue without them.

**Fix:** Check that all the joints defined in the
:hardware-interface:`robots yaml <march_hardware_builder/robots>` are defined
in the URDF that you are using from :march:`march_description/urdf <march_description/urdf>`.


.. _e119:

``E119``: Missing required key
------------------------------
A required robot config key from the robots yaml was not defined.

**Cause:** Some of the keys in the :hardware-interface:`robots yaml <march_hardware_builder/robots>`
are required to build a functional robot and the build cannot be finished without
these keys.

**Fix:** Define the missing key in the robots yaml that you are using.


.. _e120:

``E120``: URDF Initialization failed
------------------------------------
The URDF could not be loaded from the ROS parameter server.

**Cause:** The ``march_hardware_builder`` retrieves the URDF from the ROS
parameter server under the ``/robot_description`` parameter. It throws this
exception when it was not able to retrieve it, probably because was not uploaded.

**Fix:** Check if the URDF has been uploaded to the ROS parameter server. To
check the value of a ROS parameter run:

.. code::

  rosparam get /robot_description


.. _e999:

``E999``: Unknown error
-----------------------
Unknown error occurred which was not given an error code.

**Cause:** An ``HardwareException`` was thrown without specifying an ``ErrorType``.

**Fix:** Find where this exception was thrown and create a documented error.
