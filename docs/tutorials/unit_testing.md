# Unit Testing

Unit testing is a software testing method where individual components or units of a software application are tested in isolation. The primary goal of unit testing is to validate that each unit of the software performs as expected. Typically, a unit is the smallest testable part of an application, such as a function, method, or procedure. Unit testing is often conducted by developers during the development phase to identify and fix bugs early in the development cycle, ensuring the reliability and stability of the software.

## AAA structure

You can use the AAA structure to write unit tests:

1. **Arrange** – configure the test by setting up the tested system and other mechanisms.
2. **Act** – call an action to perform to test the unit.
3. **Assert** – check the result of the performed operation to verify it worked as intended.

## One Scenario per Test

A test must only test a single scenario. Testing each program part with a single scenario at a time helps identify the exact issue when a test fails.

## Unit Tests Repeatable and Scalable

To ensure an effective testing strategy, prioritize repeatability and scalability in unit tests. Encourage synchronized unit test writing alongside application code development, adopting practices like behavior-driven or test-driven programming. Emphasize combined code and test reviews to refine testing practices, even for bug fixes. Maintain a zero-tolerance policy toward test failures, addressing issues promptly to prevent time wastage and ensure robust, reliable code deployment.

## Mocking and Stubbing

Employ techniques such as mocking and stubbing to isolate the unit under test from its dependencies. By simulating the behavior of external components or services, developers can focus solely on testing the specific functionality of the unit. This approach enhances the reliability of unit tests and enables developers to identify and address potential issues within the isolated context.

1. **Mock:** In software testing, a mock is a simulated object that mimics the behavior of real objects in controlled ways, enabling the testing of a unit's interactions with its dependencies. Mocks are used to verify how a unit interacts with other units without invoking their actual implementations.

2. **Stub:** A stub is a minimal, simplified version of a module or object with predetermined behavior that is used in place of a fully functional component during testing. Stubs provide canned responses to method calls, allowing developers to isolate the unit under test from its dependencies during the testing process.

## Continuous Integration and Continuous Deployment (CI/CD)

Integrate unit testing into the CI/CD pipeline to automate the testing process. Configure automated builds and tests that run whenever changes are made to the codebase. Implementing this practice facilitates the early detection of errors, reduces the likelihood of introducing new bugs, and streamlines the development and deployment workflow.

## Collaboration and Knowledge Sharing

Promote collaboration among team members to share insights, best practices, and lessons learned from unit testing. Encourage knowledge sharing sessions, code reviews, and collaborative problem-solving to foster a culture of continuous improvement and learning within the development team.

## Policy Objective

Every time a code change is made within the software development lifecycle, it is mandatory to execute the relevant unit tests to verify that previous functionalities remain intact and unaffected.