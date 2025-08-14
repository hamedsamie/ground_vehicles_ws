# Ground Vehicles ROS 2 Workspace (Foxy)

This workspace hosts the **Ground Vehicles Software Architect** assignment.  
It follows a clean separation of concerns:
- **middleware/**: ROS 2–specific code (LifecycleNode, pubs/subs/services/actions/timers)
- **algorithm/**: pure C++ (no ROS includes)
- **state machine**: pure C++ internal operational FSM (independent from ROS lifecycle)
- **config/**: YAML parameter files (will be used in a later commit)
- **tests/**: unit & integration tests

---

## Repository layout

```text
ground_vehicles_ws/
├── README.md
└── src/
    └── ground_architect_node/
        ├── CMakeLists.txt
        ├── package.xml
        ├── include/
        │   └── ground_architect_node/
        │       ├── ground_node.hpp
        │       └── state_machine.hpp
        ├── src/
        │   ├── middleware/
        │   │   ├── ground_node.cpp
        │   │   └── main.cpp
        │   └── algorithm/
        ├── config/
        │   └── ground_node.params.yaml
        ├── launch/
        └── tests/
```

