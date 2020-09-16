.. _add-gait-label:

Add a new gait
==============
.. inclusion-introduction-start

Introduction
------------
This tutorial will teach you how to add a new gait to the codebase.
When you are done you will be able to perform the gait on the exoskeleton!

.. inclusion-introduction-end

Create new subgait files
------------------------
First create a new gait folder in the :ref:`march-gait-files-label` according to the :ref:`march-gait-files-structure-label`

Subgait files can be created with the gait generator (:ref:`using-the-gait-generator-label`) or manually.
New gaits should be added to `gait-files repository <https://github.com/project-march/gait-files>`_ with a pull request.

Create new gait file
--------------------
Once you have created your subgaits you have to define how they are connected.
Create a file named ``<your_gait_name>.gait`` inside your new gait directory.
This file is a yaml that describes the possible subgaits and transitions between
them. Add a yaml key named ``name`` and give it a name. Then create a new key
named ``subgaits`` with all subgait names as keys under it. It should also
contain the key ``start`` to indicate where the gait should start. Finally, add
transitions to the subgaits. An example is shown below.

.. code-block:: yaml

  name: walk
  subgaits:
    start:
      to: right_open
    right_open:
      to: left_swing
    left_swing:
      to: right_swing
      stop: right_close
    right_swing:
      to: left_swing
      stop: left_close
    left_close:
      to: end
    right_close:
      to: end

Add a gait button to the developer input device
-----------------------------------------------
To add a gait to the developer input device, follow :ref:`add-a-new-button-label`

Add the gait to the input device
--------------------------------
To add a gait to the input device, follow :ref:`how-to-add-a-gait-label`
