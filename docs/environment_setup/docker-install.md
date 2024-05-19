# Docker Installation

## Ubuntu

Follow the instructions in [this website](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository) under "Install using the apt repository". 
You can install Docker Desktop but at your own risk. Essentially Docker Desktop has a fancy GUI and makes some processes easier but supposedly does not run natively (runs inside VM), and might cause problems
for GUIs.

## Windows

Install [Docker Desktop on windows](https://docs.docker.com/desktop/install/windows-install/) - follow indications for WLS2 backend instead of HyperV backend, unless you don't want to use WSL2

## MacOS

Install [Docker Desktop on Mac](https://docs.docker.com/desktop/install/mac-install/)

## Docker with root

In Ubuntu, docker requires root access. To remove this, follow the instructions in [this website](https://docs.docker.com/engine/install/linux-postinstall/) to make docker available for your user, reboot your pc after the process
