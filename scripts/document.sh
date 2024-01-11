# This script requires a Doxyfile to be in the current directory, as well as pdflatex to be installed with its dependencies. 
# Refer to https://gist.github.com/rain1024/98dd5e2c6c8c28f9ea9d to install pdflatex
rm -rf ./docs/doxygen/*
mkdir -p ./docs/doxygen/localization-mapping
mkdir -p ./docs/doxygen/path_planning
doxygen ./src/loc_map/Doxyfile
doxygen ./src/planning/planning/Doxyfile
# doxygen ./src/control/Doxyfile
# doxygen ./src/perception/Doxyfile

