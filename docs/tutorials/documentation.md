# Documenting Work

In order to pass down the knowledge aquired to the next generations of driverless engineers and to make the code developed easier to understand, it is fundamental that both the code, system design and architecture and decisions are all documented.

- For code documentation, we will be using **Doxygen**. 
- To document the current structure of the system, the **documentation section** of this repository will be used. 
- To further document design decisions of the system and the whole theoretical analysis behind our decisions, **overleaf** latex documents will be used.

## Doxygen

In order to help future developers start contributing to this project, code should be easy to read and staightforward. However, this is easier said than done. For this reason, it is helpful to use a documentation tool to document the code developed. Doxygen is the de-fact tool for documentation in C++. To use it you should:
- [Install doxygen](https://www.doxygen.nl/download.html) in your machine (virtual machine in this case)
    - Download
    - Follow the compilation and installation instructions in the manual
- Create doxygen file in the project home directory
    - Run ```doxygen -g``` on the command line
    - Edit the file to your liking
        - Check the INPUT variable, for this is the one that defines what files will be read to generate the documentation
- Comment the code with doxygen comments
    - Use [this vscode extension](https://marketplace.visualstudio.com/items?itemName=cschlosser.doxdocgen) for automatic doxygen documentation generation
- Run ```doxygen``` on the root directory to generate documentation

There is an example doxyfile [here](../assets/environment_setup_tutorial/Doxyfile.example). This doxyfile is configured to generate only latex documentation (as oposed to latex + html).

The [example project](../assets/environment_setup_tutorial/training/) also contains a Doxyfile (it is the same as the one mentioned above) and a [script](../assets/environment_setup_tutorial/training/document.sh) that generates a pdf file + the latex output from doxygen. This script required the installation of [pdflatex](https://gist.github.com/rain1024/98dd5e2c6c8c28f9ea9d). The example project also contains commented code.