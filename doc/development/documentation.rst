Documentation
=============
.. inclusion-introduction-start

This tutorial will teach you how to build the documentation locally and contribute to its development.

.. inclusion-introduction-end

Introduction
^^^^^^^^^^^^
These tutorials are written in `rst <http://docutils.sourceforge.net/rst.html>`_, an easy to understand plaintext markup language.
It is then build by `Sphinx <http://www.sphinx-doc.org/en/master/>`_ using the `rosdoc_lite <http://wiki.ros.org/rosdoc_lite>`_ package.
You can either build the documentation locally when following the tutorials or deploy it to `GitHub pages <https://pages.github.com/>`_ with the help of `Travis <https://travis-ci.org>`_.

Building locally
^^^^^^^^^^^^^^^^
Follow these steps to be able to build the documentation locally.

Clone the repository
--------------------
Add the following entry to your ``.rosinstall`` file (recommended):

.. code::

  - git:
      local-name: march_tutorials
      uri: https://github.com/project-march/tutorials
      version: develop

And use wstool to update your workspace:

.. code::

  cd ~/march_ws
  wstool update -t src

Or clone the repository manually in your source directory:

.. code::

  cd ~/march_ws/src
  git clone https://github.com/project-march/tutorials

.. note:: The local repository should be called ``march_tutorials`` instead of the repository name ``tutorials`` to ensure we can uniquely access the launchfiles.

Install rosdoc_lite and Sphinx
------------------------------
We use the package rosdoc_lite to generate the documentation with Sphinx

.. code::

  sudo apt-get install ros-kinetic-rosdoc-lite


Install Gem and html-proofer
----------------------------
Gem is a package manager for Ruby, we will use it to install `html-proofer <https://github.com/gjtorikian/html-proofer>`_.
html-proofer is a tool that can validate your generated html for mistakes like broken links or missing images.

.. code::

   sudo apt-get update
   sudo apt install ruby-full

   # Check if ruby and gem got installed correctly
   ruby --version
   gem --version

   sudo gem update --system
   sudo gem install html-proofer

Install Additional dependencies
-------------------------------
Pygit is used so we can tell Sphinx what branch we are on. That way links to GitHub files can be verified against the proper branch.
This prevents html-proofer from not being able to find newly added files on develop, as it checks against the current branch.

sphinx-copybutton is used to allow you to copy the contents of code blocks to your clipboard with a single click.

Install all additional dependencies with:

.. code::

  ./install.sh

Generate the html
-----------------
Simply run the :rootdir:`build_locally <build_locally.sh>` script to generate the docs and automatically open them in your browser.

.. code::

  cd ~/march_ws/src/march_tutorials
  ./build_locally.sh

.. warning::
  If you already have sphinx installed, you might get the following error:

  .. code::

    Traceback (most recent call last):
      File "/home/march/.local/bin/sphinx-build", line 7, in <module>
        from sphinx.cmd.build import main
      File "/home/march/.local/lib/python2.7/site-packages/sphinx/cmd/build.py", line 39
        file=stderr)
            ^
    SyntaxError: invalid syntax
    stdout:

  Fix it by uninstalling sphinx

  .. code::

    pip uninstall sphinx

.. note::
  If you have added new files but not pushed to GitHub yet, html-proofer will probably complain about invalid links.
  Push your files and build locally again to solve this problem.

Browser sync (Optional)
-----------------------
`Browser sync <https://www.browsersync.io/>`_ is a tool that can refresh your browser when files get updated.
We have a python script called :rootdir:`watch.py <watch.py>` that will watch your doc files and rebuild them with Sphinx when something changes.
Browser sync is then notified and will refresh the browser.

Install it globally with npm:

.. code::

  npm install -g browser-sync

Start the python watch script:

.. code::

  cd ~/march_ws/src/march_tutorials
  python watch.py

In another terminal, start browser-sync:

.. code::

  cd ~/march_ws/src/march_tutorials
  browser-sync start -s build/html -f "build/html/*.html"

A localhost instance should now be opened, and refreshed whenever you change a ``.rst`` file.

.. note::
  Browser sync does not work perfectly, sometimes you will have to refresh the page manually,
  or change something in the file for it to start generating properly.


Deploy with Travis
^^^^^^^^^^^^^^^^^^
We make use of the `Travis deploy <http://docs.travis-ci.com/user/deployment>`_ feature to deploy our generated documentation to GitHub pages.
Please check the :rootdir:`.travis.yml of this repository<.travis.yml>` for the details.

Add a new tutorial
^^^^^^^^^^^^^^^^^^
Adding a new tutorial is as simple as creating a new ``.rst`` file.
To make sure it shows up in the Table of Contents, add it to the :rootdir:`index.rst <index.rst>` under a ``.. toctree::`` directive

.. tip:: If you are creating a new package description, make sure to base it off the :codedir:`package template <march_packages/template.rst>`