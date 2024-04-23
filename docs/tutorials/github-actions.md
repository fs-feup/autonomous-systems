# Github Actions

Github Actions is a CI/CD tool provided by Github that enables us to define automated workflows i.e. sequences of intstructions that automatically run on certain events. 
Github Actions were chosen over other tools for their growing community, ease of use and easy integration with the repository.

## Basic Definitions

- **A Workflow** is a sequence of computer commands that can be defined to run with different options and triggered by specific events related to the repository
- **A Runner** is a virtual (or real) machine that can is used to run the defined workflows. Github provides with many default runners from their servers, but you can setup you own runner using a pc you own
- **An Action** is almost like a library that can be used within a workflow to perform a certain function. These can be defined using Dockerfile or javascript programs and can be published to a market in the shape of a repository, to share with the community.
These differ from workflows in the sense that they are meant to define custom functions to be used in them and cannot directly be used

## Workflows

Workflows are the main way to implement automated events. They can be configured by a .yml file which needs to be located in the .github/workflows directory of your repository. Here is an example file with comments depicting each part's goal:

```yml
name: Testing and Building # Name to show up in the verification

on: # Events that trigger the workflow
  push:
    branches: [ "main" ]
  pull_request:
    branches: [ "main" ]

jobs: # Jobs are like processes that run in parallel on different machines

  build: # Name of the job
    runs-on: ubuntu-latest # Runner it uses

    container: # Option to use a specific docker container
      image: ros:humble-ros-base-jammy

    defaults: # Extras for selecting some default options, like the default shell
      run:
        shell: bash

    steps: # Each step to be taken, dividing into steps eases debugging and improves the look of the output
      - uses: actions/checkout@v3 # Using an action called checkout, which is fundamental as it allows the runner to checkout the current branch and commit of this repo

      - name: Install dependencies # Name of the step
        run: | 
          echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc
          ./dependencies_install.sh
      
      - name: Rosdep
        run: |
          source /opt/ros/$ROS_DISTRO/setup.bash
          rosdep update
          rosdep install --from-paths src --ignore-src -r -y
        continue-on-error: true
      
      - name: Build Messages
        run: |
          source /opt/ros/$ROS_DISTRO/setup.bash
          colcon build --packages-select custom_interfaces eufs_msgs fs_msgs

      - name: Build and Test Localization and Mapping
        run: |
          source install/setup.bash
          colcon build --packages-select loc_map
          colcon test --packages-select loc_map --event-handler=console_direct+

```

The example file sums up most important details about workflows. Workflows will, by default, output a positive check sign if all steps exit successfully or fail.
However, with some actions, personalized output can be configured and arranged.

### Using Docker Containers

As depicted in the example file, you can choose to run the workflow inside a specific docker container with the **container** option, which greatly shortens the work needed, for instance, to run ROS2 programs. 
You can also use your own container by providing a [service container](https://docs.github.com/en/actions/using-containerized-services/about-service-containers), which allows you to build a personalized docker container to help
run anything you want in the workflow. The below file gives an example on how to use this:

```yml
name: Build and Run in Custom Docker Container

on: [push, pull_request]

jobs:
  custom-build:
    runs-on: ubuntu-latest
    services:
      ros-container:
        build:
          context: . # Assuming the Dockerfile is in the same directory as the workflow file
          dockerfile: Dockerfile # Specify your Dockerfile name
        options: --name custom-ros-container

    steps:
    - uses: actions/checkout@v3

    - name: Build the Docker image
      run: docker build . -t custom-ros-image

    - name: Run commands inside the Docker container
      run: |
        docker run custom-ros-image /bin/sh -c "source /opt/ros/foxy/setup.sh && colcon build"
        # Replace `/opt/ros/foxy/setup.sh` and `colcon build` with commands suitable for your project

```

## Github Actions
Github Actions can be created to encapsulate functions to be used in workflows, simplifying them. 
They also allow you to introduce complex logic with programming languages such as javascript, which would be hard to create or impossible with shell scripts.
Visit the [github tutorials](https://docs.github.com/en/actions/creating-actions/about-custom-actions) on how to create custom actions to learn more. 

## Personalized Runners

It is possible to host your own runner i.e. run your actions in a pc of your choice. This implies setting up some definitions in that pc and in your repository. 
Follow the [official guide](https://docs.github.com/en/actions/hosting-your-own-runners/managing-self-hosted-runners/about-self-hosted-runners) for this.
