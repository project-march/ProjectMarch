.. _march-realsense-reader-label:

march_realsense_reader
======================

Overview
--------

This package subscribes the the realsense camera topics and processes one of pointclouds published there with a service callback.

This software does 4 main things:

* Preprocess the point cloud

* Find regions in the preprocessed cloud

* Create hulls from the found regions

* Find parameters with which gaits can be interpolated

The package publishes the results of all the steps for visualization in RVIZ.

These steps are enbodied by the interfaces ``Preprocessor``, ``RegionCreator``, ``HullFinder`` and ``ParameterDeterminer``.
These have implementations which execute the logic, these classes can do this in different ways.
We currently use the implementations ``NormalsPreprocessor``, ``RegionGrower``, ``CHullFinder`` and``HullParameterDeterminer``.

NormalsPreprocessor
-------------------

The idea behind this step is that a point cloud can be noisy, large, and filled with points which are not necessarily relevant.
Preprocessing aims to reduce the point cloud size and the useless areas.
We currently do not remove statistical outliers as this proved too computationally expensive where proprocessing aims to increase time efficiency.

Preprocessing consists of the following steps:

1. Randomly remove points from the point cloud.
   This is done because the pointcloud consist of almost 1 million points and we need to speed up the computation.
   Reducing the size by leaving point in a grid of a certain size is possible too, but that is more computationally expensive.

2. Transform the point cloud such that its origin is at the feet that is about to take a step and its orientation matches the orientation of this foot too.

3. Filter points which are too far away from the origin, as points far away are not relevant to the upcoming steps of the exoskeleton.

4. Estimate the normal vectors of the remaining point cloud.

5. Filter based on the normal vectors, points which we believe are part of a wall we remove as we are not capable of standing there.


RegionGrower
------------

The idea behind finding regions is that in the preprocessed cloud there are certain areas which belong together.
Regions where it is possible to place the foot, like a step of a stair. Region finding finds these regions which belong together.

We use the region growing algorithm to find regions in the cloud which is explained in detail in
`The pcl tutorial on region growing segmentation <https://pcl.readthedocs.io/projects/tutorials/en/latest/region_growing_segmentation.html>`_

CHullFinder
-----------

In order to find a potential foot location in the point cloud the found regions need to be transformed into a continuous
region where we believe it is possible to place the foot.
That is why we transform the regions into 'hulls' (a cloud detailing the edge of the region). To do this we need to:

For each region:

  1. Find a plane which fits the region (using the average normal and point)

  2. Project all the points in the region to the plane

  3. Transform the transformed points into a hull (the hull needs to be 2d), this can either create a convex or concave hull

  4. Add the found hull, its plane parameters and a vector of the indices to vectors for future use

HullParameterDeterminer
-----------------------

Now that we have hulls (or bounded planes) where where we can potentially place the foot. We can start finding a desired
foot location depending on the gait that is to be executed.

For the stairs method this is done as follows:

0) The stairs gait is interpolated from a low deep, high deep, low undeep & high undeep gait.
   We can place the foot anywhere in the convex hull of the end locations of those gaits.

1) Test a number of foot locations on the ground whether there is a potential foot location at some height. This five optional foot locations.

2) For all the optional foot locations test which ones are executable and pick a valid one which is
   closest to some ideal location (the minimal step, the average step).

3) Transform this into a parameter by finding at what percentage of the existing gait end locations the foot location is located.

For the ramp gait this is done as follows:

0) The ramp gait is interpolated from a flat & steep gait. We can place the foot anywhere in between the two ending locations of the gait.
   This is what we call the 'executable foot locations line'.

1) Test a number of foot locations on the ground whether there is a potential foot location at some height. This five optional foot locations.

2) For all the optional foot locations find which one is executable and closest to the executable foot locations line.

3) Transform this into a praameter by finding at what percentage of the executable foot locations line.


More Information On The Algorithms In Place
-------------------------------------------

For more details on the implementation see the header or the source files of the classes.

Software Architecture
---------------------

The generalized steps we are expecting from the PCL implementation, split into classes where we expect to possibly implement multiple alternatives:

.. figure:: images/pcl_software_architecture.png
   :align: center

This will be implemented in the form of an Interface for every of these 4 steps, that can be inherited by any possible implementation of these steps.
The inbetween steps will then be split into functions within this class. See below for an example for the preprocessor step:

.. figure:: images/pcl_class_structure.png
   :align: center

ROS API
-------

Nodes
^^^^^

*march_template_node* - Responsible for doing the thing.

Subscribed Topics
^^^^^^^^^^^^^^^^^

*/march/template/command* (template_msgs/TemplateCommand)
  Does the thing.

*/march/template/command/other* (template_msgs/TemplateCommand)
  Does the other thing.

Published Topics
^^^^^^^^^^^^^^^^

*/march/template/result* (template_msgs/Boolean)
  Tells you if it worked

Services
^^^^^^^^
*/march/template/do* (template_msgs/Do)
  Does something

Parameters
^^^^^^^^^^
*/march/template/counter* (*int*, required)
  How many to count
*/march/template/countings* (*int[]*, default: [])
  List of countings


Tutorials
---------

Do x
^^^^
Doing x is very easy, just do it.

Do y
^^^^
Doing y is a bit more difficult.

FAQ
---

How do I x?
^^^^^^^^^^^
Please check the tutorials.

How do I z?
^^^^^^^^^^^
z is not available at the moment.
