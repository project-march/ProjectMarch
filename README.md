# March
The main repository of the MARCH exoskeleton.

![Pipeline Status](https://gitlab.com/project-march/march/badges/main/pipeline.svg)

## C++ code style
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


# Python code Style
For code style the [pep8 style guide rules](https://www.python.org/dev/peps/pep-0008/) are followed.
To check these rules we use the [`flake8`](https://pypi.org/project/flake8/) tool and [`pep8-naming`](https://pypi.org/project/pep8-naming/) plugin.
To install run

    python2 -m pip install flake8 pep8-naming --user
    
