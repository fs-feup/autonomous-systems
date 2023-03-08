# This script requires a Doxyfile to be in the current directory, as well as pdflatex to be installed with its dependencies. 
# Refer to https://gist.github.com/rain1024/98dd5e2c6c8c28f9ea9d to install pdflatex
rm -rf ./documentation/*
doxygen ./localization-mapping/Doxyfile
doxygen ./planning/planning/Doxyfile
doxygen ./control/Doxyfile
doxygen ./perception/Doxyfile

