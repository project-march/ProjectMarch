Documentation
=============
.. inclusion-introduction-start

This tutorial will explain how to build the documentation locally,
allowing for easy contribution to the development of the documentation of the Project MARCH software.

.. inclusion-introduction-end

Introduction
^^^^^^^^^^^^
The MARCH Documentation is written in `rst <https://docutils.sourceforge.io/rst.html>`_, an easy-to-understand plaintext markup language.
It is then build by `Sphinx <https://www.sphinx-doc.org/en/master/>`_.

The documentation can either be locally after following this tutorials or it can be deployed to `Gitlab pages <https://docs.gitlab.com/ee/user/project/pages/>`_ with the help of the GitLab CI.

It is advised to always check your changes locally before deploying it to gitlab, to make sure that the changes are correct.

Building locally
^^^^^^^^^^^^^^^^
After following these steps you should be able to build the docs locally.

Clone the repository
--------------------

Clone the repository with either ssh or https:

**ssh:**

.. code::

    git clone git@gitlab.com:project-march/march.git

**https:**

.. code::

    git clone https://gitlab.com/project-march/march.git

Install Sphinx
--------------
We use the Sphinx to generate the documentation

.. code::

  sudo apt install sphinx-doc


Install Gem and html-proofer
----------------------------
Gem is a package manager for Ruby, we will use it to install `html-proofer <https://github.com/gjtorikian/html-proofer>`_.
html-proofer is a tool that can validate your generated html for mistakes like broken links or missing images.

To do that Ruby should first be downloaded through `rvm <https://github.com/rvm/ubuntu_rvm>`_ using the following commands:

.. code::

    sudo apt-add-repository -y ppa:rael-gc/rvm
    sudo apt-get update
    sudo apt-get install rvm
    sudo usermod -a -G rvm $USER
    echo 'source "/etc/profile.d/rvm.sh"' >> ~/.bashrc
    reboot

After that ruby can be installed through:

.. code::

    rvm install ruby-3.1.3

To check if ruby is installed correctly use:

.. code::

  ruby --version
  gem --version

Lastly do:

.. code::

  gem update --system
  gem install html-proofer

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

  .. code::

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
