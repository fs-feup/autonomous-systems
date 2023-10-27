# Project Rules
## Work Management
### Tasks and Backlog
For the task management, we will be using ClickUp. There will be 5 phases to a task:
- **Open:** tasks that are necessary to be completed but foreseen for other sprints
- **Sprint backlog:** tasks to be taken care of in the current sprint
- **In progress:** tasks currently being worked on
- **Blocked:** tasks that were blocked while being worked on
- **In review:** tasks that are waiting review (if a task is reviewed and changes are requested, it goes back to in progress status)
- **Rejected:** tasks in which changes were requested upon review
- **Complete:** Completed tasks

Each task should always have a reviewer assigned. Tasks should last at most one sprint. This also helps with the evaluation and reviewing of the task, as there is less to understand and grade at each time.
### Choice of Methodology
#### Sprints
A sprint is a defined interval of time to which a set of tasks is assigned and planned for. At the end of each sprint, review and reflection upon it is made and the next one is planned. The name sprint implies the shortness of the period, with the objective of being a certain agility to the work being planned and perform and thus a resilience to any change on requirements. It also allows for better tracking and evaluation of the work being developed.  
### Weekly Meeting
Hybrid (although highly advisable presencial), once a week:
#### Sprint Review
- Analyzed tasks completed
- Understand why some were not completed
- Redistribute them and possibly change them
- Possibly create new tasks
#### Sprint Retrospective
- What norms are working and which are not
- Which decisions were good and which were bad
#### Sprint Planning
- Decide tasks for the next sprint
### General Meeting
Purely descriptive. The objective is to make a general recap of what has been done on the week. All members are required to prepare their part of the presentation, following the template provided each week. The reunion is presencial.
### Follow Up Meetings
Meetings in the middle of the week on demand to check on progress and difficulties if necessary.
## Git norms
### Commits
Commits follow the [conventional commits norm](https://www.conventionalcommits.org/en/v1.0.0/#specification) (Angular convention):
```bash
git commit -m "<type>[optional scope]: <description>

[optional body]

[optional footer(s)]"
```
Example:
```bash
git commit -m "feat(mapping): implemented mapping function"
```
**Commit types:**
- feat: The new feature you're adding to a particular application
- fix: A bug fix
- style: Feature and updates related to styling
- refactor: Refactoring a specific section of the codebase
- test: Everything related to testing
- docs: Everything related to documentation
- chore: Regular code maintenance
### Branches
Trunk based development:
- **main branch:** branch where the current product is, gets updates from dev branch every sprint
- **dev branch:** branch for the sprint essentially. The objective is to perform a test run and a check at the end of each sprint before merging to the stable main branch 
- **short-lived branches:** branches used to make changes to the main branch
### Pull Requests
Committing directly to the main branch should be avoided. Instead, the creation of a new branch and the usage of a **Pull Request** is preferred because:
- with a pull request, we can have automatic test running as well as an optimized environment for code review (even if code review should be brief) before the code is included in the main project
In pull requests, it is mandatory that the author:
- Selects himself or someone else as **assignee** for the merge request (designing someone to be responsible for it)
- Selects someone within his team for **review**
- Uses a description, including “Closes #github task id on clickup” if the merge request is to **conclude a task**
In order to ease the tracking down of possible bugs introduced to the code base and to leave the commit log organized, commits should be squashed on merge and the message left on the commit should follow the standards described below:
```bash
<Pull Request name>
<commit message> e.g. docs: updated readme
```
This structure is the automatic structure defined in Squash and Merge in GitHub.
Don't forget to delete the source branches in the end, so that the project does not end up a jungle.
### Releases
There will be multiple milestones defined for the project. Most of these milestones will be associated with a release, which will map to a stable version of the software at a certain point.
## Programming
### Coding Guidelines
- New code should be often accompanied with 
    - unit and/or integration tests to cover their functionality
    - documentation (generated with doxygen for instance)
- Try to follow the SOLID principles:
    - **Singe-Responsibility:** do not construct code structures that should harness multiple responsibilities. Instead, divide it into smaller structures (multiple functions, classes, etc.)
    - **Open-closed:** software entities should be open for extension but closed for modification: try to not change code structure’s purposes and functionalities often
    - **Liskov substitution:** specifications of objects have to be capable to substitute general implementations i.e. a user of a parent class should be able to use an object of a child class instead
    - etc.
For more information on good programming practices, check these websites
- [Refactoring Guru](https://refactoring.guru)
- [SOLID](https://www.digitalocean.com/community/conceptual-articles/s-o-l-i-d-the-first-five-principles-of-object-oriented-design)
There are no restrictions to the tools that can be used to develop the code. However, some base ones:
    - Google test for unit testing
    - [Doxygen](https://www.doxygen.nl/) for code documentation in C++
### Making a Contribution - Steps
Each time a PR is going to be open and a contribution made, you should:
- run the static analysis tools to make sure the code is secure
- create tests and run all tests
- document your code and run the documentation tools
Certify yourself that you are on a new branch (unless it is an urgent bug fix).
Check [this guide](./tutorials/contribute.md) for more. 
### Evaluation
The reviewer of the Pull Request is tasked with evaluating the work developed. This task should not be taken lighly, as the reviewer will be as much at fault as the assignee for bugs or loss of quality. The reviewer should:
- Pull the code and run it to evaluate the code on its function
- Evaluate the code in its structure and eficiency by reading it
- Evaluate the work on its clarity by reading the code and other documentation developed and make sure he understands what was developed.
All pull requests should also have de department leader's approval before merge.
## Design and Investigation
Design and Investigation tasks will also be present in the management tool. The document being developed should always be attached to the task in some way.
### Latex Documents
Latex documents are to be used for investigation and other more serious documentation. They are to follow the [template](https://www.overleaf.com/project/64f9ef0ccb7ebc1387cbc673) provided and the guidelines attached to it. For design documents specifically, the chapter structure should be followed exactly as is. For others, chapters can be altered and removed/added but introduction must always be present.
### Evaluation
Most documents on design are to be evaluated by the department leader and possibly some other peer. The documents are evaluated on:
- their structure
- the content clarity
- how well the document fulfills its information goal
## Interventions with the car
Interventions in the car should follow a protocol, associated to the filling of a document (link to be added later).
