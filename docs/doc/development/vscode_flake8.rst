Add flake8 linter to VS Code
============================
.. inclusion-introduction-start

This tutorial will teach you how to add the flake8 style checks used in the pipeline, to your Visual Studio Code editor.

.. inclusion-introduction-end

Introduction
------------
The March Gitlab is configured with the so called pipeline. This pipeline performs several checks one the code we commit. One of these checks is on code style, see :ref:`style-guide`. To avoid styling errors in python, it is possible to add the flake8 linter with the checks used by the pipeline to your VS Code editor.

Installation
------------
VS Code already contains a flake8 linter by default. However, this is only the basic flake8 linter, while there are many flake8 extensions added to perform additional code styling checks. These extensions are simply python packages, which can be installed using pip. However, since the list of extensions we would like to use can differ over time (or per project), we highly recommend to install the extensions in a virtual environment, to keep your main python environment unchanged.

Creating a virtual environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
To create a virtual environment, we need to install the required python package first using:
::
    
    python3 -m pip install virtualenv

Next we can create a virtual environment. While being in the march workspace (~/march by default), open a terminal and run:
::
    
    python3 -m venv .march_venv --system-site-packages

This will create a directory called .march_venv containing the virtual python environment. The ``--system-site-packages`` argument gives the virtual environment access to the python packages already installed in your main python environment. This is necessary to use the ROS python packages we installed during the ROS installation.

Finally we can go into the virtual environment using:
::
    
    source .march_venv/bin/activate

If everything went well, you see that *(.march_venv)* is now shown in front of your bash prompt (the text shown in a terminal before the $-sign). Seeing this means that you are in the virtual python environment.

Install flake8 and extensions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
While being in the virtual environment, we can install all the flake8 packages used by the pipeline. All the packages are specified in march/requirements_flake8.txt. Therefore we can simply do:
::
    
    python3 -m pip install -I -r requirements_flake8.txt

Note that we add the -I argument to ignore already installed packages. This ensures to install flake8 in the virtual environment, even if you already installed it in your main environment.

Configure VS code
^^^^^^^^^^^^^^^^^
Now that flake8 is installed, we need to configure VS Code to use it. To do this, open VS Code and go to setting (Ctrl+,).

1. Search for ``python.linting.enabled`` and enable to enable python linting.
2. Search for ``python.linting.flake8Enabled`` and enable to enable linting with flake8.
3. Search for ``python.linting.flake8Path`` and set this to the flake8 installation directory of the virtual environment (``~/march/.march_venv/bin/flake8`` by default)
4. Search for ``python.formatting.provider`` and set this to black to use black formatting (like we do in the pipeline).

Usage
-----
Now that flake8 is installed and VS Code is configured, we can use the linter. If you open VS Code in the march project folder, the linter will automatically recognize the .flake8 file in the main directory of march, which contains a list of files and checks to ignore. The linter will check code in a python file you work on when you save. If there are styling problems, they will be mentioned in the problems tab (Ctrl+Shift+M).

If you see errors about black, it usually means that the current format does not meet the style requirements (for example problems with line length or white lines). To fix this, you can format your code automatically (Ctrl+Shift+I).

To easily go into the virtual environment the next time, you could add an alias to your bashrc file, for example ``alias svenv='source ~/march/.march_venv/bin/activate'``