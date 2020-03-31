.. _event-stream-processing-label:

Event stream Processing
=======================

Overview
--------
At March we use the `Event Stream Processing` engine from `SAS <https://www.sas.com/nl_nl/home.html>`_.
There is a lot of documentation on this engine at
`ESP documentation <https://documentation.sas.com/?cdcId=espcdc&cdcVersion=6.2&docsetId=espov&docsetTarget=home.htm&locale=nl>`_.
The engine allows for real-time data analysis and processing.

How does SAS ESP work?
----------------------
An ESP engine runs on a server, at March this is the exoskeleton. The engine can run multiple models.
A model is description of how data should be processed. :march:`This <march_data_collector/esp_models/march.xml>` is the current model used by March.
A model consists of multiple windows, which together form an acyclic graph.
There are many kinds of windows, which will be discussed later.
Data comes in at so-called `source windows`. One injection of data into a source window is called an event. A value of an event is called a field.
The other window types take in events coming from other windows (not necessarily source windows) and alter the events or create new events.

Window types
^^^^^^^^^^^^
There is really a broad range of different
`windows <https://documentation.sas.com/?cdcId=espcdc&cdcVersion=6.2&docsetId=espcreatewindows&docsetTarget=titlepage.htm&locale=nl>`_.
To get an idea on the kind of actions a window can perform a few are listed here:

Join window
    A join window takes in two event streams and outputs one event stream with fields from both input events.
    For instance at March we have a temperature sensor for each joint and a source window for each temperature sensor.
    A couple of join windows can put all the temperature values for the different joints in one event.

Aggregate windows
    An aggregate window can calculate summary statistics per group.
    A field is selected to group by. The window can then calculate things like maxima, minima, weighted averages or sums of other fields per group.
    At March we can use this to get the maximal difference in actual and desired position per gait, if we have an event stream with both the
    difference in actual and desired position and the gait name.

Compute windows
    A compute window can do a computation on an event. For instance when you have the actual and desired position
    you can compute the difference with a compute window.

Calculate windows
    A calculate window can apply an algorithm to the event it processes.
    The `list <https://documentation.sas.com/?cdcId=espcdc&cdcVersion=6.2&docsetId=espan&docsetTarget=p1iyy8xvfytolsn16djcp243wkx3.htm&locale=nl>`_
    of possible algorithms is quite long, but to name a few: Streaming Linear Regression, K Nearest Neighbour, Lag Monitoring, Image Processing and Random Forests.

Output
^^^^^^
There is no such thing as an output window in `ESP`. The idea is that you just subscribe to a window of your choosing.
This doesn't have to be a window at the end of the graph and can even be a source window.
There are a few built-in connectors, but there are also several API's. The most important subscribe types for March are listed below:

*
    One can add the csv-connector to the model to create a csv of all the events going trough the window.

*
    One can use the Python 3 module `esppy <https://github.com/sassoftware/python-esppy>`_.
    Using this with `Jupyter Lab  <https://jupyter.org/>`_ for instance can give you live information on your model.
    This can also be used for live data visualisation.

*
    One can also let the March software subscribe to a window.

Launching with `ESP`
--------------------

.. note::

    The ESP engine should be installed on the machine. You need a license for this.
    An engine is installed on the exoskeleton.

1. Launch an `ESP` server. On the exoskeleton the following terminal command is configured to start an `ESP` with the correct settings.

    .. code::

        esp_start

2. Do a normal launch (simulation, headless, normal) and set the launch argument ``esp`` to true. For instance:

    .. code::

        roslaunch march_launch march.launch esp:=true
