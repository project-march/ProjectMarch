# March Tutorials

https://docs.projectmarch.nl

[![Travis Status](https://travis-ci.com/project-march/tutorials.svg?branch=master)](https://travis-ci.com/project-march/tutorials)
![reviewdog](https://github.com/project-march/tutorials/workflows/reviewdog/badge.svg)

----------------

Welcome to the primary documentation for the March exoskeleton.

These tutorials use the [reStructuredText](http://www.sphinx-doc.org/en/stable/rest.html) format commonly used in the Sphinx "Python Documentation Generator". This unfortunately differs from the common Markdown format, but its advantage is that it supports embedding code directly from source files for inline code tutorials.

## Build Locally

If you want to test the tutorials by generating the html pages locally on your machine, use the ``build_locally`` script.

    source /opt/ros/$ROS_DISTRO/setup.bash
    ./build_locally.sh

The local website ``<LOCAL_PACKAGE_PATH>/build/html/index.html`` should automatically open in your web browser.

### Formatting and Style

**Code Formatting**

* Tutorials should exemplify best coding practices. If a contribution wouldn't pass review elsewhere, then it shouldn't pass review in the tutorials.
* Whenever possible, links should be created using the ``extlinks`` dictionary defined in ``conf.py``.
* All demo code should be runnable from within the ``march_tutorials`` package.
* Python code should be run using ``rosrun``.

**Style**

* Each tutorial should be focused on teaching the user one feature or package.
* Tutorials should flow from show to tell with videos and demos at the beginning followed by explanations.
* New tutorials should match the formatting, style and flow of existing tutorials whenever possible.

### Directory Structure

* Each tutorial should live in its own subdirectory within the `doc/` directory.
* Add your tutorial to the table of contents in `index.rst` in the root directory.
