# Project Rules

## Meetings

Same as before, once a week:

- Half of them mid sprint - just feedback and other problems
- Half of them coincide with Sprint Review and Planning - review the completed tasks, review what mistakes were made and plan next sprint

In class time will still happen, and is encouraged. It is specially a time for all the team to keep tabs on what is going on, since the discord meetings are more formal

### Sprint Review

- Analyzed tasks completed
- Understand why some were not completed
- Redistribute them and possibly change them
- Create new tasks for the product backlog

### Sprint Retrospective

- What norms are working and which are not
- Which decisions were good and which were bad

### Sprint Planning

- Decide tasks for the next sprint

## Sprints

2 week sprints, marked by the meeting

## Teams

The teams are mutable and will change throughout the project, attending to the needs of the moment. A team is a group of people attending to one area of development of the project. Code reviews are done inside each team. Each team’s tasks will be marked via label in the task.

## Tasks and Backlog

For the task management, we will be using **Gitlab’s issue board.** There will be 4 queues of tasks:

- **Product backlog:** tasks that are necessary to be completed but foreseen for other sprints
- **Sprint backlog:** tasks to be taken care of in the current sprint
- **In progress:** tasks currently being worked on
- **Done:** Completed tasks

Labels will be used to mark the area of work/team responsible for the feature. Labels can also be used for other matters if need be.

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

## Branches

Trunk based development:

- **main branch (trunk):** branch where the current product is
- **short-lived branches:** branches used to make changes to the main branch

The need for the development branch is rendered useless by the usage of ********tags******** and ******************releases:****************** in order to keep a healthy and functional project, it is important that there is some point to where the code can be rolled back to that ensures functionality of the project. This is the case because not even a great test suit and a dedicated team can completely fight off annoying bugs. However, the usage of another branch for this matter (development branch, for instance), is not necessary. Periodically, the project is subject to more intense manual testing to ensure its functionality. When the necessary conditions are met, the commit is tagged as ‘stable’ and a **release** is performed. Releases will be made once per sprint.

### Contributing - Merge Requests

Committing directly to the main branch should be avoided. Instead, the creation of a new branch and the usage of a **Merge Request** is preferred because:

- with a merge request, we can have automatic test running as well as an optimized environment for code review (even if code review should be brief) before the code is included in the main project

In merge requests, it is mandatory that the author:

- Selects himself or someone else as **assignee** for the merge request (designing someone to be responsible for it)
- Selects someone within his team for **review**
- Uses a description, including “Closes #<number of issue>” if the merge request is to **conclude a task**

In order to ease the tracking down of possible bugs introduced to the code base and to leave the commit log organized, commits should be squashed on merge and the message left on the commit should follow the standards described below:

```bash
Merge branch <origin branch> into 'main'
<commit message> e.g. docs: updated readme
See merge request <link to merge request>
```

This structure is the automatic structure defined in GitLab.

## Wiki Repository

There is another repository associated to the project’s repository: the wiki

This repository should be used for any extended documentation, such as this file or tutorials.

## Coding

- New code should be often accompanied with 
    - unit and/or integration tests to cover their functionality
    - documentation (generated with doxygen for instance)
- Try to follow the SOLID principles:
    - **Singe-Responsability:** do not construct code structures that should harness multiple responsibilities. Instead, divide it into smaller structures (multiple functions, classes, etc.)
    - **Open-closed:** software entities should be open for extension but closed for modification: try to not change code structure’s purposes and functionalities often
    - Liskov substitution: specifications of objects have to be capable to substitute general implementations i.e. a user of a parent class should be able to use an object of a child class instead
    - etc.
- There are no restrictions to the tools that can be used to develop the code. However, some base ones:
    - Google test for unit testing
    - [Doxygen](https://www.doxygen.nl/) for code documentation in C++

## Roles

- Developer: everyone
- Maintainer: a person responsible for dealing with Dev Ops related things, Backlog, and other stuff related to the platform and the development framework