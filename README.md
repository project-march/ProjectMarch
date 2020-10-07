# March
The main repository of the MARCH exoskeleton.

![Pipeline Status](https://gitlab.com/project-march/march/badges/main/pipeline.svg)

## Documentation

All documentation can be found at https://docs.projectmarch.nl

## C++ code style
All C++ code must follow the [`roscpp_code_format`](https://github.com/davetcoleman/roscpp_code_format)
code styling rules. The rules for this format are set in the `.clang-format` and the `.clang-tidy` file.
`clang-format` is a tool that automatically formats your code and `clang-tidy` perform static-analysis.
. 
Before pushing you should make sure that this is fixed, otherwise the
GitlabCI pipeline will fail. 

#####How to use
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
    
