# Setup of computer

The computational unit of the car is setup to run the certain commands on boot.
This commands are stored in the script.sh that is in /home/fsfeup.

The autonomous-systems repo is at /home/fsfeup/autonomous-systems.

To create a service to start on boot and that run the script, a unit file was created. It is the following: /etc/systemd/system/startup-setup.service

If there is a need to change this unit file do the following commands afterwards (one to reload the daemon, is needed to reload if changes are made, and the other one to make sure the service starts on boot)

``` bash
systemctl daemon-reload
systemctl enable startup-setup.service
```

To make sure it is enabled you can do 
``` bash
systemctl is-enabled startup-setup.service
```

# Connect via ssh

To connect via ssh you need to get it's ip (later it is going to be a fixed ip so you can skip this step). To get the ip of the computational unit do 
``` bash
hostname -I 
```
Then you can connect to it doing 
``` bash
ssh fsfeup@(ip got from the previous step)
```
Example:
``` bash
ssh fsfeup@192.168.0.130
```
You should be prompted with the password of the the computational unit and then you are inside the machine

# Connect via foxglove websocket

To connect via foxglove go to open a connection and then replace the localhost with the ip we got previously
Example:
``` bash
ws://192.168.0.130:8765
```
