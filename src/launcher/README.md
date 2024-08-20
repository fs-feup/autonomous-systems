# Launcher Package

## Package Information

### Description

This package contains launch files that are supposed to make your life easier when it comes to running the system.

### Launch Configurations

- `eufs_sim-testing.launch.py`: Launches the fyll system for evaluation in the eufs_sim simulator.
- `pacsim-testing.launch.py`: Launches the full system for evaluation in the pacsim simulator.
- `eufs_sim-isolated.launch.py`: Launches the eufs_sim simulator with nodes receiving simulated inputs. Should be used when nodes are supposed to be evaluated separately. You can comment the lines that launch some of the nodes so that you only evaluate one node.
- `pacsim-isolated.launch.py`: Launches the pacsim simulator with nodes receiving simulated inputs. Should be used when nodes are supposed to be evaluated separately. You can comment the lines that launch some of the nodes so that you only evaluate one node.