
.. _march_ble_ipd-label:

march_ble_ipd
==============
**Author**: Andrew Hutani, MIX

Overview
--------
This package is the implementation of the Bluetooth Input Device used to communicate with the smartwatch app, developed by MIX. This implementation **heavily** relies on the implementation from
`Bluetooth Technology for Linux Developers <https://www.bluetooth.com/bluetooth-resources/bluetooth-for-linux//>`_

This package implements a BLE Server which is basically copied one to one and altered for the use of receiving integers that correspond to exoModes.

Initialization
---------------
The first part of the constructor is quite similar to a normal ROS node, so this will mainly focus on the BLE part.

Server
^^^^^^^^^^^^^^^
The :code:`BluetoothServer` class is a Python-based Bluetooth Low Energy (BLE) server that uses the D-Bus API to interact with the BlueZ Bluetooth stack on Linux. It advertises a BLE service and characteristic, and allows a BLE client to read and write to this characteristic.

The script starts by setting up the D-Bus main loop and getting a reference to the system bus. It then sets up signal receivers to listen for property changes and interface additions on the bus.

The :code:`Advertisement` class is used to create a BLE advertisement. This advertisement is broadcasted by the server to let nearby BLE clients know of its existence and the services it offers.

The :code:`ExoModeCharacteristic` class represents a BLE characteristic that a client can read and write. It has a callback function that is called whenever a client writes to this characteristic.

The :code:`ExoModeService` class represents a BLE service that contains the :code:`ExoModeCharacteristic`. This service is added to the :code:`Application` class, which represents the GATT application that is registered with BlueZ.

The :code:`BluetoothServer` class is the main class of the script. It sets up the advertisement and the GATT application, and starts the main loop. It also handles the registration of the advertisement and the GATT application with BlueZ, and starts and stops the advertisement based on whether a client is connected. This class is a member variable of the ROS2 node.

The server runs indefinitely, waiting for BLE clients to connect and interact with the :code:`ExoModeCharacteristic`. When a client writes to the characteristic, the server prints the new value and calls the callback function which sends the .

ROS API
-------

Nodes
^^^^^
*ble_ipd_node* - Handles inputs from the smartwatch, and passes these on to the rest of the architecture


Service Client
^^^^^^^^^^^^^^
* | :code:`"get_exo_mode_array"` `GetExoModeArray <https://gitlab.com/project-march/march/-/blob/dev/ros2/src/shared/march_shared_msgs/srv/GetExoModeArray.srv>`_ 
  | Sends requested mode to the :code:`ModeMachine` and receives possible new modes
  
Bluetooth Connections
^^^^^^^^^^^^^^^^^^^^^
* The node also advertises a BLE characteristic, on which the exoModes will be received.