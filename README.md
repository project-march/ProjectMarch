# Project MARCH
![Pipeline Status](https://gitlab.com/project-march/march/badges/main/pipeline.svg)

Project MARCH is a student team of the TU Delft that is involved in the development of
a user-friendly and versatile exoskeleton, a motorized robotic suit, that can be used
to get people with a spinal cord injury to stand up and walk again. This repository
contains the code to make this happen.

## Pictures
<img src=".gitlab/readme/ivi.jpeg" alt="The exoskeleton sitting on a wooden box on a field track." width="49%">
<img src=".gitlab/readme/exo.jpeg" alt="The pilot wearing the exoskeleton and standing straight on a field track." width="49%">
<img src=".gitlab/readme/simulation.jpeg" alt="View of the backsize of a simulated exoskeleton." width="49%">
<img src=".gitlab/readme/stairs.jpeg" alt="A simulated exoskeleton walking up a ramp with a small inclination." width="49%">

## Framework
All code is built on top of the Robot Operating System (ROS). More information about ROS can be found on https://www.ros.org/about-ros/.

## Installation
- To install required ROS enviroment, please follow the [Install ROS and tools](https://docs.projectmarch.nl/doc/getting_started/install_ros_and_tools.html) instructions.
- To build and run the code, please follow the [Setup your workspace](https://docs.projectmarch.nl/doc/getting_started/setup_your_workspace.html) instructions.

## Documentation
All documentation can be found at https://docs.projectmarch.nl

## C++ code style
All C++ code must follow the [`roscpp_code_format`](https://github.com/davetcoleman/roscpp_code_format)
code styling rules. The rules for this format are set in the `.clang-format` and the `.clang-tidy` file.
`clang-format` is a tool that automatically formats your code and `clang-tidy` perform static-analysis.
. 
Before pushing you should make sure that this is fixed, otherwise the
GitlabCI pipeline will fail. 

##### How to use
First you need to install `clang-format` and `clang-tidy`:
```
sudo apt install clang-format clang-tidy
```

Now run `clang-format` from the root of this repository:
```
find . -name '*.h' -or -name '*.cpp' | xargs clang-format -i -style=file
```

Then you can run `clang-tidy` from the root of this repository:
```
find src -name '*.hpp' -or -name '*.h' -or -name '*.cpp' | xargs -L1 -P$(getconf _NPROCESSORS_ONLN) -I{} -- clang-tidy -p build {} 2> /dev/null
```


**NOTE:** Running `clang-format` can make changes to your files.
If you would like to show a diff and not use `find`, install
[`clang_format_check`](https://github.com/cloderic/clang_format_check).


# Python code Style
For code style the [pep8 style guide rules](https://www.python.org/dev/peps/pep-0008/) are followed.
To format the code we use [`autopep8`](https://pypi.org/project/autopep8/)
To check the pep8 rules we use the [`flake8`](https://pypi.org/project/flake8/) tool and [`pep8-naming`](https://pypi.org/project/pep8-naming/) plugin.
To install run

    python2 -m pip install autopep8 flake8 pep8-naming --user
    
