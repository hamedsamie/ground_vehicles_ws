# Ground Vehicles ROS 2 Workspace (Foxy)

This workspace hosts the **Ground Vehicles Software Architect** assignment.

We follow a **clean separation of concerns**:
- **middleware/** – ROS 2–specific code (LifecycleNode, publishers/subscribers, services, actions, timers)
- **algorithm/** – pure C++ (no ROS includes)
- **state machine** – pure C++ internal operational FSM (independent from ROS lifecycle)
- **config/** – YAML parameter files
- **tests/** – unit & integration tests

---

## Repository layout

```text
ground_vehicles_ws/
├── README.md
└── src/
    └── ground_architect_node/
        ├── CMakeLists.txt
        ├── package.xml
        ├── action/
        │   └── ExecuteMission.action       # custom action
        ├── include/
        │   └── ground_architect_node/
        │       ├── ground_node.hpp         # LifecycleNode declaration
        │       └── state_machine.hpp       # Internal FSM
        ├── src/
        │   ├── middleware/
        │   │   ├── ground_node.cpp         # LifecycleNode implementation
        │   │   └── main.cpp                # Node entry point
        │   └── algorithm/
        ├── config/
        ├── launch/
        └── tests/
            ├── test_state_machine.cpp      # gtest unit tests for the FSM
            └── test_params.cpp             # gtest unit tests for parameter validation
            └── test_pubsub.cpp             # gtest: pub/sub with lifecycle transitions
            └── test_services_actions.cpp   # Services + Action tests
```

## Parameters
Parameter | Type | Default | Description 
--- | --- | --- | --- 
loop_hz | int | 10 | Main loop frequency (1–100 Hz) 
verbose | bool | true | Extra logging 

## Requirements

- Ubuntu 20.04 (Focal)
- ROS 2 Foxy
- colcon, rosdep, git, cmake, build-essential

## Cloning the repository
```bash
# Go to the folder where you want to clone the repo
cd ~/

# Clone the repository
git clone git@github.com:hamedsamie/ground_vehicles_ws.git

# Go into the workspace folder
cd ground_vehicles_ws

```

## How to compile
1. **Open a terminal** and go to the workspace root:
```bash
cd ~/ground_vehicles_ws
```

2. Source ROS 2 Foxy:
```bash
source /opt/ros/foxy/setup.bash
```

3. Install dependencies
```bash
rosdep install -i --from-path src --rosdistro foxy -y
```
4. Build the workspace:
```bash
colcon build --symlink-install
```

5. Source the workspace overlay
```bash
source install/setup.bash
```

## How to test (manual + unit tests)
### Test the node manually (two terminals)
Terminal A — run the node
```bash
source ~/ground_vehicles_ws/install/setup.bash
ros2 run ground_architect_node ground_node
```

Terminal B — drive the lifecycle
```bash
source ~/ground_vehicles_ws/install/setup.bash
# Inspect initial state
ros2 lifecycle get /ground_node

# Configure (allocate publishers/timers)
ros2 lifecycle set /ground_node configure

# Activate (starts publishing; internal FSM goes READY -> RUNNING)
ros2 lifecycle set /ground_node activate

# Observe periodic status messages while active
ros2 topic echo /status

# Deactivate (pause activity)
ros2 lifecycle set /ground_node deactivate

# Cleanup (release resources)
ros2 lifecycle set /ground_node cleanup
```

Expected:
- **/status** prints running ~every 0.1s only when the node is Activated and the internal FSM is/status prints running ~every 0.1s only when the node is Activated and the internal FSM is RUNNING.
- After deactivate or cleanup, messages stop.

### Test parameter validation manually
Terminal A
```bash
# Run the node
source ~/ros2_ws/install/setup.bash
ros2 run ground_architect_node ground_node
```

Terminal B
```bash
# Drive lifecycle, then set params
source ~/ros2_ws/install/setup.bash

# Make sure the node is there and named correctly
ros2 node list
# should show: /ground_node

# 2) Configure the node 
ros2 lifecycle set /ground_node configure

# (optional) Check which params are declared now
ros2 param list /ground_node
# you should see loop_hz and verbose in the list

# 3) Set parameters successfully
ros2 param set /ground_node loop_hz 20
ros2 param set /ground_node verbose false

# 4) Activate (if you want it to start publishing)
ros2 lifecycle set /ground_node activate
```

### Manual pub/sub check
Terminal A
```bash
source ~/ground_vehicles_ws/install/setup.bash
ros2 run ground_architect_node ground_node
```

Terminal B
```bash
source ~/ground_vehicles_ws/install/setup.bash
ros2 lifecycle set /ground_node configure
ros2 lifecycle set /ground_node activate

# Watch telemetry
ros2 topic echo /telemetry
```
Terminal C
```bash
source ~/ground_vehicles_ws/install/setup.bash

# Publish a command (in another shell or the same one)
ros2 topic pub /cmd_in std_msgs/String "data: 'go'"
# Expected on /telemetry: data: "ack:go"
```

### Manual Services & Action
Terminal A - Run the node
```bash
source ~/ground_vehicles_ws/install/setup.bash
ros2 run ground_architect_node ground_node
```
Terminal B - Control lifecycle and call services
```bash
source ~/ground_vehicles_ws/install/setup.bash

# 1. Configure the node
ros2 lifecycle set /ground_node configure
# Expected output: "Transitioning successful"

# 2. Activate the node
ros2 lifecycle set /ground_node activate
# Expected output: "Transitioning successful"

# 3. Call reset service
ros2 service call /reset std_srvs/srv/Trigger "{}"
# Expected output:
# success: True
# message: "FSM reset to IDLE"

# 4. Set FSM mode to READY
ros2 service call /set_mode std_srvs/srv/SetBool "{data: false}"
# Expected output:
# success: True
# message: "Mode set to READY"

# 5. Set FSM mode to RUNNING
ros2 service call /set_mode std_srvs/srv/SetBool "{data: true}"
# Expected output:
# success: True
# message: "Mode set to RUNNING"
```
Terminal C - Send an Action goal
Services
```bash
source ~/ground_vehicles_ws/install/setup.bash

ros2 action send_goal /execute_mission ground_architect_node/action/ExecuteMission \ "{mission_id: demo, cycles: 3}" --feedback

# Expected output (feedback + result from the action server):
Waiting for an action server to become available...
Sending goal:
    mission_id: demo
    cycles: 3
Goal accepted with ID: <uuid>
Feedback:
    progress: 33%
Feedback:
    progress: 66%
Feedback:
    progress: 100%
Result:
    success: true
    message: "Mission demo completed"
Goal finished with status: SUCCEEDED
```

### Run unit tests (FSM + parameters)
From the workspace root:
```bash
# Build (if not already built)
colcon build --symlink-install

# Run tests only for this package
colcon test --packages-select ground_architect_node

# Show detailed results
colcon test-result --verbose
```



