# Bayesian Optimization

This diretory holds all necessary files to use bayesian optimization. To optimize an algorithm, create a folder with its name, for example "cone_coloring". That directory must contain a python file called "adapt.py" that defines the parameter space (the range of the parameters to be tuned; must be called "parameters_space"), and optionally define some specfic parameters to be tested (must be called "parameters_list"). That directory must contain an executable file called "bayesian_opt", which is going to be executed from the bayesian.py file. That executable must recieve the parameters through the terminal.

## Structure of "bayesian-opt"

The main folder holds the file called "bayesian.py", which is responsible for Bayesian model. It only generates the next parameters to be tested, using the Bayesian Optimization model from the library Skopt.

Each of folders inside "bayesian-opt" is related to one of the algorithm that needs tuning. 

### Structure of each algorithm's folder

This folder must hold a file called "bayesian_opt" which is an executable that recieves the parameters via terminal and outputs an evaluation for the parameters to the terminal (std::cout for C++ or print() in Python). The default bayesian optimization tries to minimize the result. If you want to maximize it, just multiply the result by -1.

This folder must hold a python file called "adapt.py" that defines the parameter space (which parameters to tune and their range), called "param_space", which the "bayesian.py" file will use. The "adapt.py" file can include a list of lists, called "parameters_list" which holds some specific sets of parameters you may want to test; if you don't want to test specific parameters, just leave an empty list there (parameters_list = []). To order in which parameters are listed is the same as in "param_space", and will be passed to the executable in that order. To make things easier, use the same order as in the launch files.

Keep everything related to each algorithm in its folder, including all necessary test scenarios and Makefiles.

## Dependencies

The the file [bayesian.py](./bayesian.py) depends on the following libraries: sys, os, signal, subprocess, importlib, skopt, and operator.
The Bayesian Optimization model is calculated by the library skopt.

## How to run

To specify which algorithm you want to tune, go to [bayesian.py](./bayesian.py) and change the variable "algorithm_to_tune" to the name of the directory corresponding to the algorithm you want to tune.

Then, you only need to run the same file, [bayesian.py](./bayesian.py), from the "home/ws/bayesian-opt" directory. Use this command:

```sh
python3 bayesian.py
```
You must do it in the Docker container.

## Result

The best 5 sets of parameters will be exported to a file called "parameters_ranking.txt" in each algorithm's directory. The order of the parameters will be the same as in "param_space" from "adapt.py".

The parameters will be appended to the previous content of the file (the previous content will not be deleted).