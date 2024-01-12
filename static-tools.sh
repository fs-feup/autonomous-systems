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
elif [ $1 = "all" ]; then
	echo "Formatting with clang-format..."
    ./scripts/clang-format.sh
	echo "Running cpplint..."
    ./scripts/cpplint.sh
	echo "Running cppcheck..."
    ./scripts/cppcheck.sh
	echo "Generating doxygen documentation..."
    ./scripts/document.sh
else
	echo "Wrong configuration. How to use: static-tools.sh <tool>"
	echo "Tool Options: 'all', 'cppcheck', 'cpplint', 'clang-format', 'doxygen'"
fi