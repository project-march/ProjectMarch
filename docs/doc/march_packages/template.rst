:orphan:

.. todo:: make sure to remove this orphan tag when copying.

.. _march-<package-name>-label:

<Package Name>
==============

Overview
--------

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