# March BLE Input Device
**Author: Andrew Hutani, MIX**
## Overview
This package is the implementation of the Bluetooth Input Device used to communicate with the smartwatch app, developed by MIX. This implementation **heavily** relies on the implementation from [Bluetooth&reg; Technology for Linux Developers](https://www.bluetooth.com/bluetooth-resources/bluetooth-for-linux/). This guide implements a BLE Server which is basically copied one to one and altered for the use of receiving integers that correspond to exoModes.

## BLE overview
Bluetooth Low Energy (BLE) is a power-efficient variant of the classic Bluetooth technology, which is designed for short-range communication between devices. BLE communication involves two types of devices: a central device and one or more peripheral devices. The central device (in our case the smartwatch) scans for and connects to peripheral devices, which advertise their presence.

Peripheral devices advertise their presence by periodically sending out packets of data. These packets can contain a variety of information, such as the device's name, its capabilities, and other data. When the smartwatch discovers a the characteristic, it can establish a connection. Once connected, the AsRock and smartwatch can exchange data.

## Node structure
### Client
- `'get_exo_mode_array'` ([GetExoModeArray.srv](https://gitlab.com/project-march/march/-/blob/dev/ros2/src/shared/march_shared_msgs/srv/GetExoModeArray.srv))

### Publishers
- `"/march/input_device/alive"` ([Alive](https://gitlab.com/project-march/march/-/blob/dev/ros2/src/shared/march_shared_msgs/msg/Alive.msg))

### Bluetooth connections
- The node also advertises a BLE characteristic

## Initialization
The first part of the constructor is quite similar to a noraml ROS node, so this will mainly focus on the BLE part.
### Server
The BluetoothServer class in the bluetooth_server.py file is a Python class that sets up a Bluetooth Low Energy (BLE) server using the D-Bus API. This server advertises itself to nearby devices and handles connections and data transfer.

In order for the Bluetooth Server to be able to signal to the node that it has received something over the GATT server, a callback function needs to be passed to the constructor. 