# This script requires a Doxyfile to be in the current directory, as well as pdflatex to be installed with its dependencies. 
# Refer to https://gist.github.com/rain1024/98dd5e2c6c8c28f9ea9d to install pdflatex
sudo rm -rf ./doxygen
doxygen
cd doxygen/latex
make
cd ..
mv latex/refman.pdf ./documentation.pdf