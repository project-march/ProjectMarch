Documentation
=============
.. inclusion-introduction-start

This tutorial will teach you how to build the documentation locally and contribute to its development.

.. inclusion-introduction-end

Introduction
^^^^^^^^^^^^
These tutorials are written in `rst <https://docutils.sourceforge.io/rst.html>`_, an easy-to-understand plaintext markup language.
It is then build by `Sphinx <https://www.sphinx-doc.org/en/master/>`_. 
You can either build the documentation locally when following the tutorials or
deploy it to `Gitlab pages <https://docs.gitlab.com/ee/user/project/pages/>`_ with the help of the GitLab CI.

Building locally
^^^^^^^^^^^^^^^^
Follow these steps to be able to build the documentation locally.

Clone the repository
--------------------

Clone the repository with either ssh or https:

**ssh:**

.. code:: bash

    git clone git@gitlab.com:project-march/march.git

**https:**

.. code:: bash

    git clone https://gitlab.com/project-march/march.git

Install Sphinx
--------------
We use the Sphinx to generate the documentation

.. code:: bash

  sudo apt install sphinx-doc


Install Gem and html-proofer
----------------------------
Gem is a package manager for Ruby, we will use it to install `html-proofer <https://github.com/gjtorikian/html-proofer>`_.
html-proofer is a tool that can validate your generated html for mistakes like broken links or missing images.

.. code::

  sudo apt update
  sudo apt install ruby-full

  # Check if ruby and gem got installed correctly
  ruby --version
  gem --version

  sudo gem update --system
  sudo gem install html-proofer

Install Additional dependencies
-------------------------------
We need some additional packages for sphinx that can be installed via pip.

.. code::

  cd ~/march/docs
  pip install -r requirements.txt

.. _install-pandoc-label:

Install Pandoc
--------------
We need pandoc to convert the markdown files to rst files. Pandoc is used by the
:rootdir:`build_locally <build_locally.sh>` script to turn the README files in the
src of ros1 and ros2 into rst files. To install pandoc run:

.. code::

  sudo apt update && sudo apt install -y pandoc


Generate the html
-----------------
Run the :rootdir:`build_locally <build_locally.sh>` script to
generate the docs and automatically open them in your browser.

.. code::

 cd ~/march/docs
 bash ./build_locally.sh

.. note::
  If you have added new files but not pushed to GitLab yet, html-proofer will probably complain about invalid links.
  Push your files and build locally again to solve this problem.

.. note::
  If you get the error:

  .. code-block::

    Warning, treated as error:
    /home/[user]/march/docs/index.rst:109:toctree contains reference to document 'doc/march_packages/doc/README' that doesn't have a title: no link will be generated

  This is because you don't have pandoc installed. See :ref:`install-pandoc-label`



sphinx-autobuild
----------------
`sphinx-autobuild <https://pypi.org/project/sphinx-autobuild/>`_ is a tool that
watches your doc files and live updates your changes.

You can install it with pip:

.. code::

  pip3 install --user sphinx-autobuild

Start the auto build:

.. code::

  sphinx-autobuild . build/html

When you go to ``localhost:8000`` it should open the documentation and live refresh
when a file is changed and saved to disk.

Deploy with GitLab CI
^^^^^^^^^^^^^^^^^^^^^
We make use of the `GitLab CI <https://docs.gitlab.com/ee/ci/>`_  to deploy our generated documentation to GitLab pages.
Please check the :rootdir:`.gitlab-ci.yml of this repository <.gitlab-ci.yml>` for the details.

Add a new tutorial
^^^^^^^^^^^^^^^^^^
Adding a new tutorial is as simple as creating a new ``.rst`` file.
To make sure it shows up in the Table of Contents, add it to the :rootdir:`index.rst <index.rst>` under a ``.. toctree::`` directive

.. tip:: If you are creating a new package description, make sure to base it off the :codedir:`package template <march_packages/template.rst>`
