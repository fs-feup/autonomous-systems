# Autonomous Systems Bible

# Commandments

1. **Thou shalt not generate brutal Pull Requests with dozens of files changed**
2. **Thou shalt not submit work for review in the last hours of the sprint**
3. **Thou shalt always focusing on reviewing colleagues work when possible**
4. **Thou shalt read all READMEs, the project rules and guides**
5. **Thou shalt not attempt everything alone: if stuck, share your problem with thy colleagues and communicate**
6. **Thou shalt always respect thy colleagues**
7. **Thou shalt be able to take criticism and change thy product**
8. **Thou shalt not kill thy colleagues in grappling**
9. **Thou shalt document thy learnings, as to allow future colleagues to embark in this journey**
10. **Thou shalt not abandon thy colleagues, be assiduous and come to the room**
11. **Thou shalt not give up**
12. **Thou shalt communicate with thy team**
13. **Thou shalt take thy work's deadlines seriously**

# Index

1. Mind Map
2. Links
3. Project Management Methodology and Project Rules
4. Coding Rules
5. Documentation Rules
6. Procedure Rules

# Mind Map

# ![](https://t9012047031.p.clickup-attachments.com/t9012047031/1e0f7f9a-bd35-494b-9452-564fcfed382d/MindMap%20AS.png)Links

[https://calendar.google.com/calendar/u/0?cid=OTBhNzYxZTY3NzA5MjNhMjExNzgyNTFlYmZlOTFkOTU3MDZhOTU5MDQwYTRmODFhZmMwYTZiMDIxZDdkYzM5YUBncm91cC5jYWxlbmRhci5nb29nbGUuY29t](https://calendar.google.com/calendar/u/0?cid=OTBhNzYxZTY3NzA5MjNhMjExNzgyNTFlYmZlOTFkOTU3MDZhOTU5MDQwYTRmODFhZmMwYTZiMDIxZDdkYzM5YUBncm91cC5jYWxlbmRhci5nb29nbGUuY29t)

*   [Github](https://github.com/fsfeup-driverless/driverless): Code repository and code development documentation.
*   [Drive](https://drive.google.com/drive/u/2/folders/1NpfttWcQSCiDOUB7L9gVngY0Oo6YHROh): Theoretical knowledge base, general purpose documents.
*   90120111765 ([https://app.clickup.com/9012047031/v/s/90120111765/1](https://app.clickup.com/9012047031/v/s/90120111765/1)): Project Management
*   **Discord:** communication and meetings

# Project Management Methodology

## Subsystems - Area Division

The division in subsystems is ruled by the division that maximizes the differences in knowledge areas for each subsystem. This division is performed with the same goal of the division in departments: focus on specialization to maximize results (not everybody can know everything). That division originates the following structure:

*   Perception ➝ exteroceptive sensor and processing software
*   Localisation and Mapping ➝ state estimation software, motion related sensors
*   Planning and Control ➝ Control theory and definition of the controllers for the system
*   Simulation, Validation and Integration ➝ Integration software, CAN, adapters, simulation and mechanics

## Time Division

### Phases

The work is divided into phases. Phases exist to allow the team and department's objectives to be divided into a time order. This division allows for a cyclical procedure of planning, review and learning. There are 3 types of phase:

1. Concept - the objective is to increase the team's base of knowledge and for the members to learn the theory themselves:
    1. define the main objectives for the system
    2. investigate software and hardware and document it extensively
    3. define main components and high-level architecture, as well as main hardware and software to be used
2. Development - develop the system - most common.
3. Integration - integrate the system in the vehicle (much work in debugging, trial and error).

At the beginning of each phase, all Epics that compose it are created and planned.

Each phase also culminates in:

*   a major documentation review
*   Great retrospective
*   review of all the work done (close to the end)

Phases should be a few months long:

*   not so long that major parts of the system go unreviewed
*   not so short that no relevant work is developed

At the end of each development phase, a release is created.

#### Notes

*   Much of the documentation and investigation is done in concept phases, but definitely not all
*   Integration procedures also occur in other phases
*   Code development also occurs in other phases

### Epics

A block is a unit of time to which epics are assigned. At the beginning of a block, the tasks that constitute the block are created. The block is a not a very relevant definition, as it is not a fixed interval of time and simply denotes the realization of an epic. Epics equate to a certain block of tasks that result in the completion of a concrete mission. Epics exist so that the phase's objectives are easier to map to tasks and progress can be better evaluated, originating phases for evaluation of a greater amount of tasks than the sprints would allow for.

While epics in other phases only aggregate a greater amount of tasks of the same type, development phase epics are divided into 5 steps:

1. Conceptualization ➝ create an idea of the program to be developed
2. Design ➝ define requirements, goal variables and create base diagrams
3. Code Development ➝ develop the code and test it (can be more than one PR, each PR has to contain code and unit tests).
4. Evaluation ➝ develop extra tests and use evaluation and simulation tool to validate the component
5. Documentation ➝ document the code; describe decisions made and explain program in latex document; create more detailed diagrams

### Tasks and Sprints

A sprint is a defined interval of time to which a set of tasks is assigned and planned for. The name sprint implies the shortness of the period, with the objective of giving a certain agility to the planning of work and thus a resilience to any change on requirements. It also allows for better tracking and evaluation of the work being developed.

A task is a unit of work planned for a sprint. Tasks should last at most one sprint. A task becomes too big when its details cannot be foreseen in its planning. This also helps with the evaluation and reviewing of the task, as there is less to understand and grade at each time.

#### Task Types

Tasks have different types types that require different methods of realization and review, and therefore different rules:

*   Documentation (includes investigation)
*   Code development
*   Procedural e.g. actions with the car, tests with sensors, etc.

#### Task Creation

In principle, only the department leader is to create tasks. Nevertheless, the rules are:

*   Include description as per template with reviewer and general intent of the task
*   Include dependencies that that task has with other tasks
*   Include subtasks
*   Include checklist with objectives and requirement
*   Set priority

At the beginning of the sprint:

*   Set the reviewers as watchers
*   Set the assignees
*   Set the start and due date
*   Discuss on the sprint points

Epics also contain a template in ClickUp.

#### General Rules on Tasks

*   Tasks should be finished until Saturday at 23:59 (roughly two days before end of sprint). If they are not, an update and justification should be left in the tasks' chat, mentioning everyone affected by it.
*   Frequent updates on the task status are encouraged, including updates on subtasks status and update comments on the chat (one update mid sprint at least).
*   When a task is ready for review, it should be moved to the corresponding status and reviewers should be tagged.

### Sprint Points

Sprint points are a way of measuring the effort necessary to conclude a task. Sprint points will be used to measure the capacity of the department and to allocate a suitable amount of work weekly. Every department meeting, everyone will decide together what is the amount of sprint points a task could have. After a few weeks, the average **speed** of the team is known and allocation of tasks and planning is made easier.

### Subtasks

A task is subdivided in subtasks, which are units of work indicated to be completed without interruption. They exist for the sole purpose of the pleasure of ticking boxes (and for increased traceability, division when necessary and clarity of the steps a task takes)

## Board

For the task management, we will be using ClickUp. There will be 5 phases to a task:

*   **Open:** tasks that are necessary to be completed but foreseen for other sprints
*   **Sprint backlog:** tasks to be taken care of in the current sprint
*   **In progress:** tasks currently being worked on
*   **Blocked:** task depends on other not complete tasks
*   **For review:** tasks that are waiting review
*   **Rejected:** the reviewer requested changes in the task
*   **Complete:** Completed tasks

### General Rules

*   Only the department leader is responsible for populating the board.
*   Tasks are moved to sprint backlog according to the planning made in the beginning of the week.
*   Everyone should keep an eye on the 'For Review' status, in order to be able to review work effectively and live people blocked the least possible.
*   Reviewers move tasks to rejected.
*   Assignees are responsible for the rest of the transitions.
*   Department Leader is the last reviewer, when there is more than one.

## Reviews

Each task should always have two reviewers assigned: department leader and another person.

### Assigning a reviewer

The second reviewer is assigned according to the following rules:

1. For code development, the reviewer should be familiar with the code, preferably from the same or similar subsystem
2. For other tasks, a more flexible manner of reviewer choice can be employed, using data such as workload on the week, convenience for the reviewer to get in touch with that information or skills/experience of the reviewer.

### How to perform a review

Notes on the review should be mostly given through the chat of the task in question in ClickUp. Alternatively, the review can be performed in Github, but this should be notified in the ClickUp task as well. Notes can be demands or questions/suggestions. Even for demands, there is the possibility of the assignee defending their point and rejecting the demand. It is, therefore, intended to be a debate as much as possible.

#### Code review

*   Look at the code developed
    *   Ensure its understandibility
    *   Ensure that is is well designed and well written
    *   Ensure that is fulfills its goal
*   Pull the code, compile it, run it and run the tests and ensure it performs the intended function

#### Documentation Review

*   Look at the text developed
*   Ensure its understandibility
*   Ensure that is is well structured and well written
*   Ensure it fulfills its information goal

#### Procedural Review

*   Check the report of the procedure is according to what was defined
*   Check it is well written

The review should be made through ClickUp chat, assigning the task's assignee to any changes requested. In case of approval, a ✅ should be used in the chat. If there are changes requested, the task should be moved to rejected status. Reviews can be made in other places, including in person, but a mention to this interaction should be left in the task's chat.

### General Rules on Review

*   Do not submit tasks for review at the end of the sprint, leave enough time for a reviewer to be able to check your work and for you to apply changes accordingly before the end of the sprint.
*   Subtasks reviews are also possible but not necessary. They make the most sense in the case of a task using multiple pull requests. In that case, the task is allowed to go into Review before full completion.

## Meetings

### Department Meeting

Weekly, on room B320:

*   Random matters
*   Sprint Review
    *   Analyzed tasks completed
    *   Understand why some were not completed -> write on lessons book
    *   Redistribute them and possibly change them
    *   Possibly create new tasks
*   Sprint Retrospective (every 2 weeks or so, no need for every week)
    *   What norms are working and which are not
    *   Which decisions were good and which were bad
*   Sprint Planning
    *   Decide tasks for the next sprint
    *   Allocate sprint points to tasks
    *   Define reviewers

### General Meeting

Weekly, on room B218:

Purely descriptive. The objective is to make a general recap of what has been done on the week. All members are required to prepare their part of the presentation, following the template provided each week. The reunion is in person.

### Follow Up Meetings

Meetings in the middle of the week to check on progress and difficulties and debate on unforeseen details of the task.

### Department Review Meetings

At the middle-point of a phase, a review between department members occurs where each subsystem and member present

*   the work developed
*   the **choices made**
*   the results obtained

After that, a discussion ensues on points to be improved and adjustments to be made. Its goal is to promote overall knowledge of the system by the department members and create an environment for some final adjustments to be made to the work.

### Phase End - Start Meeting

At the end of each phase, a larger Department meeting occurs where:

*   Great Restrospective occurs - a larger scale retrospective
*   The order of works and changes for the next phase are presented

### Phase End Preparation Meeting

Same as department review meeting, but with the intent of originating last changes to the work done before the phase ends. Should be done 1-2 weeks before phase end.

### Team Review

Same as department review, but includes all team and focuses on the most important details only.

# Code Development Rules

## Git norms

### Connecting to ClickUp

Each task in ClickUp contains an ID next to a GitHub symbol. All activities in Github connected to a task should contain the #id of the task(s) in a commit / pull request. Additionally, if intended and applicable, after the id, an intended status change can be indicated, as follows: `git commit -m "#id[complete]"`

### Commits

Commits follow the \[conventional commits norm\]([https://www.conventionalcommits.org/en/v1.0.0/#specification](https://www.conventionalcommits.org/en/v1.0.0/#specification)) (Angular convention):

```bash
git commit -m "<type>[optional scope]: <description>

[optional body]

[optional footer(s)]"
```

**Example:**

```bash
git commit -m "feat(mapping): implemented mapping function"
```

**Commit types:**

*   feat: The new feature you're adding to a particular application
*   fix: A bug fix
*   style: Feature and updates related to styling
*   refactor: Refactoring a specific section of the codebase
*   test: Everything related to testing
*   docs: Everything related to documentation
*   chore: Regular code maintenance

If a commit fits into multiple types:

```plain
git commit -m "fix-docs(loc-map): ..."
```

**Commit scopes:**

*   loc\_map / l\_m
*   planning\_control / p\_d
*   perception
*   simulation
*   general
*   integration
*   ...

### Branches

Trunk based development:

*   \*\*main branch:\*\* branch where the current product is, gets updates from dev branch every sprint
*   \*\*dev branch:\*\* branch for the sprint essentially. The objective is to perform a test run and a check at the end of each sprint before merging to the stable main branch
*   \*\*short-lived branches:\*\* branches used to make changes to the main branch

### Pull Requests

Committing directly to the main branch should be avoided. Instead, the creation of a new branch and the usage of a **Pull Request** (should be called Merge request, because it is a request for a merge, but oh well) is preferred because:

*   with a pull request, we can have automatic test running as well as an optimized environment for code review (even if code review should be brief) before the code is included in the main project

In merge requests, it is mandatory that the author:

*   Selects himself or someone else as **assignee** for the merge request (designating someone to be responsible for it)
*   Selects someone within his team for **review**
*   Includes the ClickUp tasks' ids in the description for the ones that are associated with it

#### Other rules

*   In order to ease the tracking down of possible bugs introduced to the code base and to leave the commit log organized, commits should be squashed on merge and the message left on the commit should follow the standards described below:

```plain
<Pull Request name>
	<commit message> e.g. docs: updated readme
```

This structure is the automatic structure defined in Squash and Merge in GitHub.

*   Don't forget to delete the source branches in the end, so that the project does not end up a jungle.
*   Create short Pull Requests, more than one per task if necessary

### Releases

At the end of each development phase, a stable version of the system is created, and a release is made in consequence, marking a point of the system that is stable and can be consulted later.

## Programming

**Mandatory:** follow the [startup guide](https://github.com/fsfeup-driverless/driverless/blob/dev/docs/tutorials/startup_guide.md). This will teach you how to perform the tasks necessary for development and also outlines the rules and guidelines in more detail.

### Coding Guidelines

*   New code should be often accompanied with
    *   unit and/or integration tests to cover their functionality
    *   code documentation (doxygen)
*   Try to follow the SOLID principles:
    *   **Single-Responsibility:** do not construct code structures that should harness multiple responsibilities. Instead, divide it into smaller structures (multiple functions, classes, etc.)
    *   **Open-closed:** software entities should be open for extension but closed for modification: try to not change code structure’s purposes and functionalities often
    *   **Liskov substitution:** specifications of objects have to be capable to substitute general implementations i.e. a user of a parent class should be able to use an object of a child class instead
    *   etc.
*   Use c++ function instead of c function as they are less safe.
*   Do not use normal prints for logging, use ROS logging tools and utilize different levels of logging for the effect.

For more information on good programming practices, check these websites

\- [Refactoring Guru](https://refactoring.guru)

\- [SOLID](https://www.digitalocean.com/community/conceptual-articles/s-o-l-i-d-the-first-five-principles-of-object-oriented-design)

There are no restrictions to the tools that can be used to develop the code. However, some base ones are mandatory:

\- Google test for unit testing in C++ and Unittest for Python (come with ROS2)

\- [Doxygen](https://www.doxygen.nl/) for code documentation in C++ and Python

### Making a Contribution - Steps

Each time a PR is going to be open and a contribution made, you should:

*   run the static analysis tools to make sure the code is secure
*   create tests and run all tests
*   document your code and run the documentation tools

Certify yourself that you are on a new branch (unless it is an urgent bug fix).

Check [this guide](https://github.com/fsfeup-driverless/driverless/blob/dev/docs/tutorials/contribute.md) for more.

# Documentation Rules

Design and Investigation tasks will also be present in the management tool. The document being developed should always be attached to the task in some way.

## Diagrams

Diagrams should follow UML guidelines and be done in [draw.io](http://draw.io). The source .drawio files should be included in the repository or in the folders the exported images are. Pay special attention to:

*   Sequence diagrams: should strictly follow rules and be specially aligned with the code, as they can give insight on the code but only if they are precise and exact
*   Class diagrams: do not necessarily need to be very complete but should also follow UML rules pretty tightly
*   Flow (control/data flow / activity): more flexible

Diagrams are created for two intents:

*   Before development - design of a system (do not need to be so precise)
*   After development - often improvements of the latter ones

Consult the [slides on UML from André Restivo](https://web.fe.up.pt/~arestivo/page/) and [UML.org](https://www.uml.org/) website.

## Latex Documents

Latex documents are to be used for investigation and other more serious documentation. They are to follow the [template](https://www.overleaf.com/read/zyphdsdpscph#a5f10f) provided and the guidelines attached to it. For design documents specifically, the chapter structure should be followed exactly as is. For others, chapters can be altered and removed/added but introduction must always be present.

### Rules and Guidelines on Documents

Most rules are already present in the template's introduction. However, some more detailed ones:

*   The objective of the documents is to describe information for other people (and ourselves) consult later. Therefore, it aims to transmit the most information possible the quickest way possible. This implies:
    *   write the least possible to illustrate an idea, just enough to cover all points of it (meaning do not cut on the content if it explains something else, but fancy words are unnecessary)
    *   write well and clearly: write short phrases, use visual items, images, tables and other structures that can help interpretation
*   Be careful about the compilation warnings and errors: these slow down compilation and often inform on some real problem
*   Be careful with underscores and other special symbols, they break everything
*   Make sure to add all sources! Compile every source of information you used that was useful and add it to the bibliography (and cite it accordingly)
*   Include glossary with key words and definitions
*   Write an abstract in the end, that sums up all parts of the document

# Procedure Rules

Interventions with the car and other components (LiDARs) should follow a protocol, associated to the filling of a document:

[https://docs.google.com/document/d/162pLJhoCZ34lnNz9KEsf8aD15CitbIg0h7naqDMJCGE/edit](https://docs.google.com/document/d/162pLJhoCZ34lnNz9KEsf8aD15CitbIg0h7naqDMJCGE/edit)