# Gait Generation
Robot agnostic rqt plugin which allows the user to create joint trajectories.

![GitHub release (latest by date including pre-releases)](https://img.shields.io/github/v/release/project-march/gait-generation?include_prereleases)

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
