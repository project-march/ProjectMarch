.. _using-the-pressure-soles-label:

Using the pressure soles
========================
At March we currently use the `Moticon <https://www.moticon.de/>`_ pressure soles. Live reading of the data from the pressure soles is very cumbersome. You need the following things:

* The pressure soles with charged batteries.
* An Android phone with the Moticon app installed.
* A Windows laptop with the Moticon desktop software installed.
* A computer with the March software installed.

All devices should be connected to the same WiFi for optimal functioning.

1. First you open the app on the phone and pair the insoles with the phone.

.. note::

    The pressure soles are turned on by moving them, because the internal IMU is triggered.
    Shake the pressure soles when they do not automatically turn on.

2. Next you should set the record mode to live capture. Open the Moticon desktop app and go to the record section and settings. Set the following things:

    * UDP Input:
        - `Channel name:`           Joint_positions
        - `Number rof Channels:`    8
        - `Datatype:`               float32
        - `Port:`                   9999

    * UDP Output
        - `IP addess:`              \<exoskeleton_ip\>
        - `Port:`                   8888
        - `Filter:`                 force pressure cop

More info on this step can be found `here <https://www.moticon.de/doc/science_desktop_software/record/udp/>`_.

3. Lastly use should set the correct launch arguments. You should set pressure_soles to true and if you are not using the
standard Switch laptop connected to the project March router, you should set the IP-address on which the Moticon desktop app is running.
