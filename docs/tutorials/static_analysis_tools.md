# Static Analysis Tools

This tutorial mentions the multiple static analysis tools setup in this project and teaches the developer how to use them.

**NOTES:** 
- All the tools are already installed in the devcontainer. If you are not using the container, you should consult the installation commands in the [Dockerfile](../../.devcontainer/Dockerfile).
- We used to use a few separate tools configured to run in Pull Requests such as CPPCheck and CPPLint. We have moved to use Sonar tools.

## Formatters

We use two tools for formatting code:
- **black** for python
- **clang-format** for C++

Both can be run by using the [format.sh script](../../format.sh), but they are also configured to run on file save by the [vscode settings.json file](../../.vscode/settings.json).

## Sonarcloud

Sonarcloud is a static analysis services. It is configured in our project to analyze our code and generate reports on it. It runs upon push to main or pull request. 

### Access
When in a Pull Request, a short summary is provided as a comment:

![PR Comment](../assets/static_analysis_tutorial/pr_comment.png)

You can access more details about the report by clicking in any link of the ones in the comment. You can also access the Sonarcloud platform [via link](https://sonarcloud.io/organizations/fs-feup/projects?sort=name), which is also in the Notion department's home page.

### Home Page
The project's home page at Sonarcloud looks like this:

![home page](../assets/static_analysis_tutorial/home.png)

The projects of the team are listed in the projects tab, along with metrics regarding code quality. Other tabs include:

- **Quality Profiles:** define groups of rules that are active for different languages
- **Rules:** showcase the specific rules the code must comply with
- **Quality Gates:** define limits that pull requests must fulfill to be considered valid, such as minimum code coverage or maximum bugs

The other tabs shown are only present if you have an administrator account in sonarcloud.

### Project
Upon entering a project, there are four main views:

- **Overview:** provides a general view on the project
- **Main Branch:** details on the main branch
- **Pull Requests:** open pull requests and details on them
- **Branches:** details on other analyzed branches

Both the branches and each pull request provide a similar view, which enables an in-depth analysis on the quality of the code and the detected issues:

- **Summary:** quick summary on the state and problems of the code in that branch or PR
- **Issues:** shows information on all the issues regarding that branch or PR
    ![Issues](../assets/static_analysis_tutorial/issues.png)
    The issues presented were raised by static analysis tools from Sonar itself, to verify code quality on multiple dimensions:
    - **Type:** Code Smell, Bug or Vulnerability
    - **Severity:** High, Medium or Low
    - **Software Quality:** Reliability, Security and Maintainability
    
    Among others...

    It also allows advanced search and filter capabilities
- **Security Hotspots:** similar to issues, but focuses on code that could present security vulnerabilities specifically.
- **Measures:** provides information on the state of the branch of PR through insightful plots and statistics
- **Code:** allows for issues and bugs to be searched via file, by presenting a directory structure
- **Activity:** irrelevant mostly

### Issues and Security Hotspots

Each issue and security hotspot is accompanied by information regarind its location and cause, reasoning behing the classification, rules used and sometimes even possible fixes (specific fixes for that situation, on top of the general suggestions provided in the *Where is this issue?* and *Why is this an issue?* tabs).

![Issue](../assets/static_analysis_tutorial/issue.png)

## Sonarlint

Sonarlint is an extension for VSCode that allows for immediate static analysis on code development. 

### Set Up

The tool must be configured before it can be used:
1. Connect sonarlint to the organization in Sonarcloud
    ![Connecting Sonarlint to Sonarcloud](../assets/static_analysis_tutorial/lint-cloud-connection.png)
    1. Create an account linked to your github account
    2. Click in generate token
        ![Generate new token](../assets/static_analysis_tutorial/token.png)
    3. Add your organziation key
        ![Organization key](../assets/static_analysis_tutorial/key.png)
    4. Add connection name
    5. Save connection
    6. Select a project repository
2. Provide sonarlint with a compile_commands.json, which allows it to understand how the C++ code works and how to analyze it.
    1. Make sure you have bear installed (already installed in docker environment)
    2. Run the compilation command with bear:
        ```sh
        bear -- colcon build
        ```

        The command will generate a *compile_commands.json* file which is automatically read by Sonarlint. You can use bear multiple times (or all times) you compile, to ensure the linter works well.


### Features

After set-up, Sonarlint will show the same issues present in Sonarcloud with error squiggles, providing a simplified version of the content. Example:
1. ![Sonarlint error step 1](../assets/static_analysis_tutorial/sonarlint1.png)
2. ![Sonarlint error step 2](../assets/static_analysis_tutorial/sonarlint2.png)
3. ![Sonarlint error step 3](../assets/static_analysis_tutorial/sonarlint3.png)

### Final Notes

- Sometimes, Sonarlint will stop detecting issues for some reason. If this happens, run the bear command again, originating a new *compile_commands.json* file.
