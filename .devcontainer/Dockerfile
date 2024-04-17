FROM ros:humble-ros-base-jammy
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
RUN apt install ros-$ROS_DISTRO-rviz2 -y
RUN apt install openssh-client -y
RUN apt install openssh-server -y

# Install some useful tools
RUN apt update
RUN apt install python-is-python3 -y
RUN pip install rosbags
RUN apt install clang-format -y
RUN pip install black
# For Sonarlint
RUN apt install bear -y
RUN curl -s https://deb.nodesource.com/setup_16.x | sudo bash && apt install nodejs -y
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc
ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]