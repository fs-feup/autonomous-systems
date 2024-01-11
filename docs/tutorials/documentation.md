# Documenting Work

In order to pass down the knowledge aquired to the next generations of driverless engineers and to make the code developed easier to understand, it is fundamental that both the code, system design and architecture and decisions are all documented.

- For code documentation, we will be using **Doxygen**. 
- To document the current structure of the system, the **documentation section** of this repository will be used. 
- To further document design decisions of the system and the whole theoretical analysis behind our decisions, **overleaf** latex documents will be used.

## Doxygen Setup

In order to help future developers start contributing to this project, code should be easy to read and staightforward. However, this is easier said than done. For this reason, it is helpful to use a documentation tool to document the code developed. Doxygen is the de-fact tool for documentation in C++. To use it you should:
- [Install doxygen](https://www.doxygen.nl/download.html) in your machine (virtual machine in this case). Download the binaries version.
    - Download
    - Run ```sudo make install``` inside
- Create doxygen file in the project home directory (when creating a new project, that is)
    - Run ```doxygen -g``` on the command line
    - Edit the file to your liking
        - Check the INPUT variable, for this is the one that defines what files will be read to generate the documentation
- Comment the code with doxygen comments
    - Use [this vscode extension](https://marketplace.visualstudio.com/items?itemName=cschlosser.doxdocgen) for automatic doxygen documentation generation
- Run ```doxygen``` on the [src folder](../../src/) to generate documentation
    **IMPORTANT:** In this project, there is already a script that runs documentation on all packages automatically [here](../../scripts/document.sh). This functionality is also included in [this script](../../static-tools.sh).

T
## Doxygen C++

C++ code example:
```c++
/**
 * @brief Class for Kalman Filter
 * Used for SLAM
 * 
*/
class KalmanFilter {

 public:
  /**
   * @brief Base constructor
   * 
   * @param MotionModel motion_model motion model to be used
   * @param ObservationModel observation_model validation model to be used
   * 
  */ 
  KalmanFilter(const MotionModel& motion_model, const ObservationModel& observation_model);
}

```


## Doxygen Python

After following the [Doxygen Setup guide](#doxygen-setup), you need to make a few changes to the default *Doxyfile* so that the Doxygen command correctly parses python code:

- Enable `JAVADOC_AUTOBRIEF`, changing its value from `NO` to `YES`. This allows Doxygen to utilize Javadoc style briefs which look something like this:

```py
##
# Brief summary description of the particular element.
#
# Detailed description of the particular element
# that includes much more information.
```

- To optimize the generated documentation for Java and Python based source code, change the `OPTIMIZE_OUTPUT_JAVA` to `YES`.

- Tell Doxygen to generally extract all elements found in the source code, `EXTRACT_ALL` to `Yes`.

   - Note, the `EXTRACT_PRIVATE` and `EXTRACT_STATIC` settings can also be set to `YES` if you want to include private class members and static file members in the generated documentation as well.

- Activate `HIDE_SCOPE_NAMES` to `YES`, hiding the scope name that is typically prepended to element names contained within that scope.

- To sort the elements in alphabetical order instead of when they are declared, set `SORT_BRIEF_DOCS` to `YES`.

These are just some enhancements to the extraction and readability of the generated documentation. In reality, only `OPTIMIZE_OUTPUT_JAVA` and `INPUT` are required.

Supposing you have already changed the `INPUT` to find the Python source code, you are all set to start generating Python documentation with Doxygen.

### Comment blocks in Python

For Python there is a standard way of documenting the code using so called documentation strings `"""`. However when using these strings none of doxygen's special commands are supported and the text is shown as verbatim text.

To have the doxygen's special commands and have the text as regular documentation instead of `"""` use `"""!` or set `PYTHON_DOCSTRING` to `NO` in the configuration file.

```py
"""!
@package docstring
Documentation for this module.
 
More details.
"""
 
def func():
    """!
    Documentation for a function.
 
    More details.
    """
    pass
 
class PyClass:
    """!
    Documentation for a class.
 
    More details.
    """
   
    def __init__(self):
        """!
        The constructor.
        
        @param self Param documentation
        """
        self._memVar = 0
   
    def PyMethod(self):
        """!
        Documentation for a method.
        
        @return The return value
        """
        pass
```
