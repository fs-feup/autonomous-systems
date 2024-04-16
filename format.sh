#!/bin/sh
echo "Formatting with clang-format..."
./scripts/clang-format.sh
echo "Formatting with black..."
black src/