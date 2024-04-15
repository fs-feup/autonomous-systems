# Static Analysis Tools

This tutorial mentions the multiple static analysis tools setup in this project and teaches the developer how to use them.

**NOTES:** 
- All the tools are already installed in the devcontainer
- In our system, all tools are configured to be run through a script called [static-tools.sh](../../static-tools.sh)

## C++

### CPPCheck

In this project, we will be using CPPCheck as a static analysis tool for the c++ packages, to check for any underlying bugs. We will be using it in the GitHub pipeline, but it is handy that the developer can use it locally. To install it, just run ```sudo apt install cppcheck``` (to install in windows, check [their website](https://cppcheck.sourceforge.io/))

To use it, run:

```sh
./static-tools.sh cppcheck
```

### CPPLint

CPPLint will be used to check some aditional linting and styling on the C++ packages of the project. To install it, use [pip](https://pypi.org/project/pip/): ```pip install cpplint``` (you will therefore require python and pip).

CPPLint can be configured through configuration files called 'CPPLINT.cfg'. These files can be present in multiple directories and configurations stack with subdirectories (unless 'set noparent' is used). 

To run CPPLint, execute the following command:

```sh
./static-tools.sh cpplint
```

### Clang-format

CLang-format is a formatter i.e. it automatically formats code following a certain suite of guidelines provided in a configuration file (like clang-format.txt). This will be used to format c++ code, maintaining similar code style throughout the codebase.

To install Clang-format in a debian-based system, run ```sudo apt install clang-format```.

```sh
./static-tools.sh clang-format
```

## Python

### Black

Black is a straightforward formatter widely used for its simplicity. To use it, run:

```sh
./static-tools.sh black
```

Black is not very configurable, but it is extremely fast and does the job well enough.

### Pylint

Pylint is one of the most used python static analysis tools, overall due to its configurability and robustness. The enumerous options to edit are chown in a .pylintrc file (among other options). This python linter is extremely complete, providing insights in all sorts of good coding behaviours and all sorts of errors.
To run it:

```sh
./static-tools.sh pylint
```

### Mypy

Mypy is a small addition to the team. It is a simpler linter that focuses on type checking, which is great as python does not have intrinsic type checking, even if typed python is growing due to its better readability and ease of development. It essentially checks if the types defined for the variables were indeed correct or not, ignoring the cases where a type was not given. To run it:

```sh
./static-tools.sh mypy
```

## Static tools and scripts

To make the usage of these tools easier, scripts have been configured to make them work out of the box. The main script, **static-tool.sh** is essentially a wrapper around the configured and complex commands for each tool. On top of the configurations presented, the script can also be ran with the following options:
- **all** - run all tools
- **format** - run formatting tools (black and clang-format)
- **check** - run analysing tools (cpplint, cppcheck, pylint and mypy)

Another important note is that these tools are configured to run in an automated github workflow, which analyses the repo's code every push or PR to main.