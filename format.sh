#!/bin/sh
echo "Formatting with clang-format..."
find ./src -regex '.*\.\(cpp\|hpp\|cc\|cxx\)' -exec clang-format -style=file -i {} \;
echo "Formatting with black..."
black src/