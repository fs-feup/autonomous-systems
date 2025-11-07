# Autonomous Mobile Robot System

## Perception, State Estimation, Planning, and Control

This project implements a (incomplete) pipeline for an autonomous mobile robot, encompassing **Perception**, **State Estimation**, **Path Planning**, and **Control**, realized as four independent ROS 2 nodes. The system enables the robot to explore an unknown environment, build a map, localize itself, and execute a delivery mission by picking up boxes and placing them on corresponding shelves.

---

## 1. System Architecture and Files

The system is structured into four distinct ROS 2 packages/nodes, each responsible for a specific stage of the robot's autonomy pipeline. In order to run the project, the **robot.launch.py** file deals with launching all nodes needed -> "ros2 launch launcher robot.launch.py"

| Node Name | File | Primary Responsibility | Key Subscriptions | Key Publications |
|-----------|------|------------------------|-------------------|------------------|
| **PerceptionNode** | `perception_node.cpp` | Simulates perception radius and object detection. | `robot/map`, `robot/status` | `perception/objects`, `perception/local_map` |
| **StateEstimationNode** | `state_estimation_node.cpp` | Builds a map with the information gathered by perception. | `robot/status`, `perception/objects` | `state_estimation/map`, `state_estimation/debug` |
| **PathPlanningNode** | `path_planning_node.cpp` | Tries to plan the movement to detect the full map and then tries to plan the movement to pick up all boxes and drop them in the respective shelfs. | `state_estimation/map`, `state_estimation/debug` | `planning/path` |
| **ControlNode** | `control_node.cpp` | Executes the planned path into discrete commands. | `planning/path`, `state_estimation/debug`, `robot/status` | `control/cmd`, `control/status` |

---

## 2. Techniques and Algorithms Used

###  State Estimation (Localization and Mapping)

#### Localization

- **Dead Reckoning**: The robot's position (`robot_grid_x_`, `robot_grid_y_`) is primarily updated based on the velocity information received from the `/robot/status` topic.

- **Map Alignment (Global Reference)**: The discovery of the first bottom-left wall corner is used to establish a global (world) coordinate system origin (0,0), equal to the one used in the actual map, allowing the robot's position to be reported in a consistent world frame, not just relative to the start position 'S'. This variable is not used later in the project.

#### Mapping

- **Occupancy Grid**: An internal 101×101 grid (`std::vector<std::string> grid_`) is used to store the incrementally built map.

- **Sensor Fusion**: Detections from the Perception node are projected onto the grid based on the estimated robot position, marking known objects ('1'-'5' for boxes, 'A'-'E' for shelves, '9' for walls, '0' for floor).

- **Exploration Marking**: Cells within the `perception_radius` are marked as '0' (floor) if they were previously '?' (unknown), effectively clearing the known navigable space.

###  Path Planning

- **A\* Search Algorithm**: The core pathfinding engine is implemented using A* search. This guarantees finding the shortest path in the grid, making the path efficient for the Control node to execute. I had help from AI code generators in order to build this feature.
  - **Heuristic**: The Euclidean distance is used as the heuristic h (admissible and consistent for grid movement).

- **Two-Phase Mission Logic**: The robot operates in two sequential phases:
  - **Exploration**: The robot prioritizes finding and traveling to the nearest frontier cell (a known, walkable cell adjacent to an unknown '?' cell) until the map is fully explored (`isMapFullyExplored` uses a BFS to check connectivity). This was not finished.
  - **Delivery**: After exploration, the robot switches to finding the shortest total travel distance for the combined (Robot → Box → Shelf) path for any matched Box-Shelf type (e.g., box '1' to shelf 'A'). This was not finished or tested.

### Control

- **Discrete Movement**: The Control node translates the A* planned path (a sequence of grid points) into discrete, atomic commands: UP, DOWN, LEFT, RIGHT, and future PICK/DROP.

- **Closed-Loop Control with Timeout**: After sending a movement command, the node waits for position confirmation by checking if the robot's estimated grid position (`robot_pos_`) matches the `expected_pos_`. A 2-second timeout is implemented to handle potential movement failures or delays, preventing the robot from getting stuck.

---

## 3. Key Design Decisions

- **Grid-Based World**: All internal processing in State Estimation and Path Planning uses a discrete grid representation, which simplifies A* implementation and movement commands (1 unit = 1 grid step).

- **Perception Abstraction**: The PerceptionNode's use of a configurable `perception_radius` successfully enforces the constraint of local, relative sensing, preventing the State Estimation node from accessing global map information.

- **State Estimation Decoupling**: The State Estimation node only relies on velocity updates and relative perception data, strictly adhering to the "no cheating" rule by building the map and localizing without access to the simulator's true global state.

- **Priority-Based Planning**: The Exploration-First approach ensures that the robot has the most complete map possible before attempting the final delivery mission, maximizing the chance of finding all required boxes and shelves.

---

## 4. Performance and Solution Highlights

- **Robust State Estimation**: The system successfully handles localization updates purely from velocity and accurately maps objects into the global grid using relative perception data. The dynamic 'R' marking for the robot's current position on the map ensures the Path Planner always starts from the correct location.

- **Reliable Control Loop**: The implemented movement confirmation with a timeout mechanism is critical for robustness in a simulated or real-world environment where commands might be dropped or movement might fail, preventing perpetual waiting.

---

## 5. References and Resources

- ROS 2 Documentation and Tutorials for package creation, topics, and timers.
- A* Search Algorithm resources (specifically for grid-based pathfinding).
- Frontier-Based Exploration literature for autonomous mobile robots.
