#!/bin/sh
if [ $1 = "cpplint" ]; then
	echo "Running cpplint..."
    ./scripts/cpplint.sh
elif [ $1 = "cppcheck" ]; then
	echo "Running cppcheck..."
    ./scripts/cppcheck.sh
elif [ $1 = "clang-format" ]; then
	echo "Formatting with clang-format..."
    ./scripts/clang-format.sh
elif [ $1 = "doxygen" ]; then
	echo "Generating doxygen documentation..."
    ./scripts/document.sh
elif [ $1 = "ruff" ]; then
	echo "Running ruff..."
    ./scripts/ruff.sh
else
	echo "Wrong configuration. How to use: static-tools.sh <tool>"
	echo "Tool Options: 'cppcheck', 'cpplint', 'clang-format', 'doxygen', 'ruff'"
fi