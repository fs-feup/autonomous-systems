#!/bin/sh
if [ $# = 0 ]; then
	echo "Wrong configuration. How to use: static-tools.sh <tool>"
	echo "Tool Options: 'all', 'act' (format and document), 'check' (analyzers)\n'cppcheck', 'cpplint', 'clang-format', 'doxygen'"
elif [ $1 = "cpplint" ]; then
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
elif [ $1 = "act" ]; then
	echo "Formatting with clang-format..."
    ./scripts/clang-format.sh
	echo "Generating doxygen documentation..."
    ./scripts/document.sh
elif [ $1 = "check" ]; then
	echo "Running cpplint..."
    ./scripts/cpplint.sh
	echo "Running cppcheck..."
    ./scripts/cppcheck.sh
fi