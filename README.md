<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
# March
The main repository of the MARCH exoskeleton.

![GitHub release (latest by date including pre-releases)](https://img.shields.io/github/v/release/project-march/march?include_prereleases)

| Branch | Build Status |
| ------ |:------------:|
| master | [![Build Status](https://api.travis-ci.com/project-march/march.svg?branch=master)](https://travis-ci.com/project-march/march) |
| develop | [![Build Status](https://api.travis-ci.com/project-march/march.svg?branch=develop)](https://travis-ci.com/project-march/march) |

## Fixing code style
All C++ code must follow the [`roscpp_code_format`](https://github.com/davetcoleman/roscpp_code_format)
code styling rules. The rules for this format are set in the `.clang-format`
file. `clang-format` is a tool that can detect and fix these problems in your
code. Before pushing you should make sure that this is fixed, otherwise the
Travis build will fail. First you need to install `clang-format`:
```
sudo apt install clang-format
```
Then you can run `clang-format` from the root of this repository:
```
find . -name '*.h' -or -name '*.cpp' | xargs clang-format -i -style=file
```
**NOTE:** This command can make changes to your files.

If you would like to show a diff and not use `find`, install
[`clang_format_check`](https://github.com/cloderic/clang_format_check).
=======
# Simulation
The simulation of the exoskeleton.

![GitHub release (latest by date including pre-releases)](https://img.shields.io/github/v/release/project-march/simulation?include_prereleases)

| Branch | Build Status |
| ------ |:------------:|
| master | [![Build Status](https://api.travis-ci.com/project-march/simulation.svg?branch=master)](https://travis-ci.com/project-march/simulation) |
| develop | [![Build Status](https://api.travis-ci.com/project-march/simulation.svg?branch=develop)](https://travis-ci.com/project-march/simulation) |

>>>>>>> simulation/develop
=======
# monitor
Monitor RQt plugins for the March exoskeleton

![GitHub release (latest by date including pre-releases)](https://img.shields.io/github/v/release/project-march/monitor?include_prereleases)
 [![codecov](https://codecov.io/gh/project-march/monitor/branch/master/graph/badge.svg?flag=production)](https://codecov.io/gh/project-march/monitor)

| Branch | Build Status |
| ------ |:------------:|
| master | [![Build Status](https://api.travis-ci.com/project-march/monitor.svg?branch=master)](https://travis-ci.com/project-march/monitor) |
| develop | [![Build Status](https://api.travis-ci.com/project-march/monitor.svg?branch=develop)](https://travis-ci.com/project-march/monitor) |
>>>>>>> monitor/develop
=======
# gait-files
A collection of gait files to run on the march-iv exoskeleton
>>>>>>> gait-files/develop
=======
# Hardware Interface
The hardware interface of the MARCH exoskeleton. This includes EtherCAT master and uses the SOEM library.

![GitHub release (latest by date including pre-releases)](https://img.shields.io/github/v/release/project-march/hardware-interface?include_prereleases)
[![codecov](https://codecov.io/gh/project-march/hardware-interface/branch/develop/graph/badge.svg?flag=production)](https://codecov.io/gh/project-march/hardware-interface)


| Branch | Build Status |
| ------ |:------------:|
| master | [![Build Status](https://api.travis-ci.com/project-march/hardware-interface.svg?branch=master)](https://travis-ci.com/project-march/hardware-interface) |
| develop | [![Build Status](https://api.travis-ci.com/project-march/hardware-interface.svg?branch=develop)](https://travis-ci.com/project-march/hardware-interface) |

>>>>>>> hardware-interface/develop
=======
# Gait Generation
Robot agnostic rqt plugin which allows the user to create joint trajectories.

![GitHub release (latest by date including pre-releases)](https://img.shields.io/github/v/release/project-march/gait-generation?include_prereleases)
[![codecov](https://codecov.io/gh/project-march/gait-generation/branch/master/graph/badge.svg?flag=production)](https://codecov.io/gh/project-march/gait-generation)

| Branch | Build Status |
| ------ |:------------:|
| master | [![Build Status](https://api.travis-ci.com/project-march/gait-generation.svg?branch=master)](https://travis-ci.com/project-march/gait-generation) |
| develop | [![Build Status](https://api.travis-ci.com/project-march/gait-generation.svg?branch=develop)](https://travis-ci.com/project-march/gait-generation) |

# Code Style
For code style the [pep8 style guide rules](https://www.python.org/dev/peps/pep-0008/) are followed.
To check these rules we use the [`flake8`](https://pypi.org/project/flake8/) tool and [`pep8-naming`](https://pypi.org/project/pep8-naming/) plugin.
To install run

    python2 -m pip install flake8 pep8-naming --user
    
Then you can run `flake8` from the root and it will give errors you should fix, otherwise the Travis build will fail.
>>>>>>> gait-generation/develop
