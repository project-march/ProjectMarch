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
      local-name: tutorials
      uri: https://github.com/project-march/tutorials
      version: develop

And use wstool to update your workspace:

.. code::

  wstool update -t src

Or clone the repository manually in your source directory:

.. code::

  https://github.com/project-march/tutorials

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
   ruby --version
   gem --version
   gem update --system
   sudo gem install html-proofer

Install pygit2
--------------
Pygit is used so we can tell Sphinx what branch we are on. That way links to GitHub files can be verified against the proper branch.
This prevents html-proofer from not being able to find newly added files on develop, as it checks against the current branch.

.. code::

  pip install pygit2 --user

Generate the html
-----------------
Simply run the :rootdir:`build_locally <build_locally.sh>` script to generate the docs and automatically open them in your browser.

.. code::

  ./build_locally

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

  python watch.py

In another terminal, start browser-sync:
.. code::

  browser-sync start -s build/html -f "build/html/*.html"

A localhost instance should now be opened, and refreshed whenever you change a ``.rst`` file.

.. note::
  Browser sync does not work perfectly, sometimes you will have to refresh the page manually,
  or change something in the file for it to start generating properly.


Deploy with Travis
^^^^^^^^^^^^^^^^^^
We make use of the `Travis deploy <http://docs.travis-ci.com/user/deployment>`_ feature to deploy our generated documentation to GitHub pages.
Please check the :rootdir:`.travis.yml of this repository<.travis.yml>` for the details.

Style guide (TODO)
^^^^^^^^^^^^^^^^^^
.. todo:: (Isha) add a style guide.
