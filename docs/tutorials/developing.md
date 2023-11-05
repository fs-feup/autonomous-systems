# How to develop

This tutorial will teach you the steps behind developing a component / feature for this system. It also links sources that will teach you how to develop elegant and well built programs - a well built program is understood as a program that is coded in the most computationally efficient way possible without compromising readability and extensibility (the ability of a program to be extended).

## Steps to contruct a component

- **Conceptualize** the program
- **Design** the program
- **Code** the program (includes the creation of unit tests)
- **Evaluation** of the program with simulators and other tools
- **Documentation** of the program

## Concept

The concept of a component is often easy to establish on the basis of the larger concept of the program. However, the idea is to idealize in rough terms what the program will be, what problem it will solve and how it will do it vaguely. It includes the research of methodologies and routes available and is motly a note taking and learning process that occurs before the component's design.

## Design

The design of a program includes:
1. the definition of the program's requirements
2. the definition of the program's objectives
3. the definition of a testing method
4. the design of the program's architecture and behaviour

For the design of programs, **UML** (Unified Modeling Language) is used. UML is a language to create schematic models of programs to both depict their structure and behaviour. There are many different types of diagrams, which can all be found [here](https://creately.com/blog/diagrams/uml-diagram-types-examples/). Each type of diagram has its own rules, which should be followed as much as possible.

The diagrams created in the design step should not be very detailed and accurate, but should still respect the rules and be coherent. To learn how to create these diagrams:
- [UML.org](https://www.uml.org/)
- [State Diagram Slides](https://web.fe.up.pt/~arestivo/slides/?s=uml-state#1)
- [Class Diagram Slides](https://web.fe.up.pt/~arestivo/slides/?s=uml-classes#1)
- [Sequence Diagram Slides](https://web.fe.up.pt/~arestivo/slides/?s=uml-sequence#1)
- [Communication Diagram Slides](https://web.fe.up.pt/~arestivo/slides/?s=uml-communication#1)

## Coding

Coding envelops both the creation of code, its documentation through comments and the creation of unit tests accordingly. The tests can be created a priori for a test oriented programming style.

### SOLID
In order to code our program in a elegant manner, there are principles we must follow: the SOLID principles: 
- **Single-Responsibility:** do not construct code structures that should harness multiple responsibilities. Instead, divide it into smaller structures (multiple functions, classes, etc.)
- **Open-closed:** software entities should be open for extension but closed for modification: try to not change code structure’s purposes and functionalities often
- **Liskov substitution:** specifications of objects have to be capable to substitute general implementations i.e. a user of a parent class should be able to use an object of a child class instead
- **Interace segregation:** clients should not be forced to depend upon interfaces that they do not use. Meaning, create specific interfaces (S)
- **Dependency inversion:** high-level modules should not depend on low-level modules, they should both depend on abstractions

Read [André Restivo's slides](https://web.fe.up.pt/~arestivo/slides/?s=solid#1).

### Design Patterns

For many common functions, design patterns have been listed and created for programmers to follow. They are essentially recipes to fulfill a certain need for a program. Check [André Restivo's slides](https://web.fe.up.pt/~arestivo/slides/?s=patterns#1)

### Code Smells and Refactoring

Code smells are signs that a certain portion of code is rotting (it smells), meaning it is node well designed and programmed. Refactors are techniques and mechanism used to better the code and solve these problems. For knowledge on these, check [André Restivo's slides](https://web.fe.up.pt/~arestivo/slides/?s=refactoring#1) on the matter. You can also check the [Refactoring Guru's page](https://refactoring.guru/) for an extensive list on code smells, refactors and design patterns.

## Evaluation

The evaluation should be done using simulation, experimentation and other tools available to evaluate the performance of a system and ensure its functionality. 

## Documentation

Documentation should:
- start from the design previously developed
- complete the diagrams and completement with others
- define in greater detail the requirements and objectives of the component
- describe the logic behind the program with precision and concision

## Last notes

Each of these steps is roughly taken for every epic but also for some smaller features and components. It is often an iterative process, as not always the first try at developing a program is successful. 

It might also be useful to know the C++ language in detail. For that, go through [The Cherno's tutorials](https://www.youtube.com/watch?v=18c3MTX0PK0&list=PLlrATfBNZ98dudnM48yfGUldqGD0S4FFb&index=1). 