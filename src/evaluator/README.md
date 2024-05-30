# Evaluator Package

This package contains the evaluator node, used to evaluate the AS main pipeline, with aid of simulation environments and other data.

## Package info 

- **Package name:** evaluator
- **Node name:** evaluator

### Launch files
- **evaluator-eufs.launch.py** - evaluator with adapters for EUFS SIM
- **evaluator-pacsim.launch.py** - evaluator with adapters for PacSim
- **evaluator-fsds.launch.py** - evaluator with adapters for FSDS


## Design

The structure of the node is quite simple and depicted by the following class diagram:

![Class Diagram](../../docs/diagrams/sim-inf/evaluator-class-diagram.drawio.svg)

The way the system works can be explained with the aid of the following sequence diagram:

![Sequence Diagram](../../docs/diagrams/sim-inf/evaluator-sequence-diagram.drawio.svg)

The Evaluator node class only creates Subscribers for the topics intrinsic to the system. The subscriptions and subsequent linkage to callbacks is all made in the adapters, as they depend of the simulator topics. The callbacks in the adapters simply organize and format the inputs to a standard format, so that metrics can be calculated and published by the evaluator node and its functions. 

This is true for all but the control node, which for simplicity reasons sends all the information necessary for evaluation in a separate message, as it does not use simulation data to evaluate itself.

The concept of the complete evaluation system can be seen in the following diagram:

![System Diagram](../../docs/diagrams/sim-inf/evaluation-system.drawio.svg)

## Results

The metrics will be published in ```/evaluator/*``` topics.