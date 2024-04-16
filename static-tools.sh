#!/bin/sh
if [ $# = 0 ]; then
	echo "Wrong configuration. How to use: static-tools.sh <tool>"
	echo "Tool Options: 'all', 'format', 'check' (analyzers)\n'cppcheck', 'cpplint', 'clang-format',\n'mypy', 'pylint', 'black'"
elif [ $1 = "cpplint" ]; then
	echo "Running cpplint..."
    cpplint --recursive ./src 
elif [ $1 = "cppcheck" ]; then
	echo "Running cppcheck..."
    ./scripts/cppcheck.sh
elif [ $1 = "clang-format" ]; then
	echo "Formatting with clang-format..."
    ./scripts/clang-format.sh
elif [ $1 = "mypy" ]; then
	echo "Running mypy..."
    mypy --config-file ./scripts/mypy.ini src/
elif [ $1 = "pylint" ]; then
	echo "Running pylint..."
    pylint --rcfile=./scripts/.pylintrc src
elif [ $1 = "black" ]; then
	echo "Formatting with black..."
    black src/
elif [ $1 = "all" ]; then
	echo "Formatting with clang-format..."
    ./scripts/clang-format.sh
	echo "Running cpplint..."
    cpplint --recursive ./src 
	echo "Running cppcheck..."
    ./scripts/cppcheck.sh
	echo "Formatting with black..."
    black src/
	echo "Running pylint..."
    pylint --rcfile=./scripts/.pylintrc src
	echo "Running mypy..."
    mypy --config-file ./scripts/mypy.ini src/
elif [ $1 = "format" ]; then
	echo "Formatting with clang-format..."
    ./scripts/clang-format.sh
	echo "Formatting with black..."
    black src/
elif [ $1 = "check" ]; then
	echo "Running cpplint..."
    cpplint --recursive ./src 
	echo "Running cppcheck..."
    ./scripts/cppcheck.sh
	echo "Running pylint..."
    pylint --rcfile=./scripts/.pylintrc src
	echo "Running mypy..."
    mypy --config-file ./scripts/mypy.ini src/
fi