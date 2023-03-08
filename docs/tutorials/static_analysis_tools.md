# Static Analysis Tools

This tutorial mentions the multiple static analysis tools setup in this project and teaches the developer how to use them.

## CPPCheck

In this project, we will be using CPPCheck as a static analysis tool for the c++ packages, to check for any underlying bugs. We will be using it in the GitHub pipeline, but it is handy that the developer can use it locally. To install it, in the Virtual Machine, just run ```sudo apt install cppcheck``` (to install in windows, check [their website](https://cppcheck.sourceforge.io/))
To use it, run ```cppcheck --enable=all --error-exitcode=1 src/ -i test/```. This command already has some tunning to ignore some things and not ignore others. To learn more, run ```cppcheck -h```.

There is already a [script](../../src/cppcheck.sh) in the source folder called **cppcheck.sh** for this effect.

## CPPLint

CPPLint will be used to check some aditional linting and styling on the C++ packages of the project. To install it, use [pip](https://pypi.org/project/pip/): ```pip install cpplint``` (you will therefore require python and pip).

CPPLint can be configured through configuration files called 'CPPLINT.cfg'. These files can be present in multiple directories and configurations stack with subdirectories (unless 'set noparent' is used). 

CPPLint does not have the possibility of defining full directories to be ignored in the check. As such, we need to define in the run command what directory should be analyzed. To run CPPLint in the current directory, execute the following command:

```sh
cpplint --recursive ./
```
In the src folder there is a [script](../../src/cpplint.sh) that already runs this command called **cppint.sh**.

## Clang-format

CLang-format is a formatter i.e. it automatically formats code following a certain suite of guidelines provided in a configuration file (like clang-format.txt). This will be used to format c++ code, maintaining similar code style throughout the codebase.

To install Clang-format in a debian-based system, run ```sudo apt install clang-format```.

To run, the command is quite complex, as the program does not have the capability to be ran on entire directories. A [script](../../src/clang-format.sh) as been created in the src folder called **clang-format.sh**. This script can be used to run the formatter on all c++ files in src.