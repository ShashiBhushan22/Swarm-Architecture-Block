# Version 1 Architecture - Quick Reference Guide

**Last Updated:** December 25, 2025

---

## 📋 Document Structure

This package contains:
1. **VERSION_1_ARCHITECTURE.md** (2100+ lines) - Comprehensive detailed specification
2. **VERSION_1_FLOWCHART.drawio** - Visual system architecture
3. **QUICK_REFERENCE_V1.md** (this file) - Fast lookup guide

---

## 🎯 Mission: Area Scanning with Drone Swarm

**Version 1 Goal:** Build a functional end-to-end system that can:
- Scan a designated area with one or more drones
- Build a map from scan data
- Identify points of interest
- Allocate tasks to drones
- Plan collision-free paths
- Execute autonomous missions

---

## 🏗️ System Architecture (9 Modules)

### 1. **ORCHESTRATOR** 
- **Role:** Central coordinator, state machine manager
- **Key Function:** Sequences workflow across all modules
- **States:** INIT → READY → SCANNING → MAPPING → POI → ALLOCATION → PLANNING → EXECUTION → COMPLETED

### 2. **SCANNING** 
- **Role:** Generate and execute area coverage plan
- **Algorithm:** Grid pattern (V1)
- **Output:** Sensor data (images, point clouds)

### 3. **MAPPING** 
- **Role:** Build occupancy grid from sensor data
- **Algorithm:** Ray-casting, connected components
- **Output:** 2D occupancy grid + obstacle list

### 4. **POINTS OF INTEREST (POI)** 
- **Role:** Identify locations requiring inspection
- **Algorithm:** Rule-based detection (obstacles, coverage gaps)
- **Output:** Prioritized POI list

### 5. **TASK ALLOCATION** 
- **Role:** Assign POIs to drones
- **Algorithm:** Greedy nearest-neighbor (V1)
- **Output:** Per-drone task assignments

### 6. **PATH PLANNING** ⭐
- **Role:** Generate collision-free paths
- **Algorithm:** A* with safety margin inflation
- **Processing:** Sequential for-loop, immediate publish
- **Output:** Waypoint path per drone

### 7. **COMMAND GENERATION** 
- **Role:** Convert paths to flight commands
- **Algorithm:** Waypoint sequencing with timing
- **Output:** MAVLink-compatible command sequence

### 8. **DGRC BRIDGE** 
- **Role:** ROS2 ↔ MAVLink translation
- **Technology:** MAVROS
- **Output:** Bidirectional telemetry and commands

### 9. **TELEMETRY & PX4** 
- **Role:** Interface with PX4 autopilot
- **Telemetry Rate:** 10-20 Hz
- **Output:** Position, velocity, battery, status

---

## 📊 Key Data Structures

### Path Planning Request
```python
{
    "drone_id": "D1",
    "start_position": {"x": 0.0, "y": 0.0, "z": 10.0},
    "goal_position": {"x": 50.0, "y": 50.0, "z": 10.0},
    "occupancy_grid": {...},
    "constraints": {"safety_margin": 1.0, "max_altitude": 50.0}
}
```

### Path Planning Response
```python
{
    "drone_id": "D1",
    "path": [
        {"position": {"x": 0, "y": 0, "z": 10}, "yaw": 0, "timestamp": 0},
        {"position": {"x": 10, "y": 10, "z": 10}, "yaw": 45, "timestamp": 3.3},
        # ... more waypoints
    ],
    "status": "success",
    "path_length": 70.7,  # meters
    "planning_time": 2.3  # seconds
}
```

### POI Structure
```python
{
    "poi_id": "POI_001",
    "position": {"x": 25.0, "y": 30.0, "z": 15.0},
    "type": "obstacle_inspection",
    "priority": 7,  # 1-10
    "estimated_inspection_time": 30.0  # seconds
}
```

---

## 🔧 Key Algorithms (V1)

### A* Path Planning
```python
def plan_path_astar(start, goal, grid):
    # 1. Initialize open/closed lists
    # 2. Add start to open list
    # 3. While open list not empty:
    #    - Pop node with lowest f = g + h
    #    - If goal, reconstruct path and return
    #    - Explore 8-connected neighbors
    #    - Update costs and parent pointers
    # 4. Apply safety margin inflation
    # 5. Smooth path (optional)
    return path
```

### Greedy Task Allocation
```python
def allocate_tasks(pois, drones):
    # 1. Sort POIs by priority (high to low)
    # 2. For each POI:
    #    - Find nearest available drone
    #    - Check battery constraint
    #    - Assign POI to drone
    # 3. Optimize sequence per drone (nearest neighbor)
    return assignments
```

### Grid Scan Pattern
```python
def generate_grid(area, spacing, altitude):
    # 1. Generate parallel lines N-S direction
    # 2. Alternate direction (lawnmower pattern)
    # 3. Add turnaround points
    # 4. Optimize to minimize turns
    return waypoints
```

---

## 🚀 Quick Start Commands

### Launch Full System (Simulation)
```bash
# Terminal 1: PX4 SITL
cd ~/PX4-Autopilot
make px4_sitl gazebo

# Terminal 2: ROS2 System
cd ~/swarm_ws
source install/setup.bash
ros2 launch swarm_system full_system.launch.py

# Terminal 3: Mission Execution
ros2 run swarm_system mission_commander \
  --mission scan_area \
  --area "0,0,100,100" \
  --drones 3
```

### Monitor System Status
```bash
# Check all nodes
ros2 node list

# Monitor telemetry
ros2 topic echo /drone/D1/pose --once

# Visualize in RViz
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix swarm_system)/share/config/swarm.rviz
```

### Debug Individual Modules
```bash
# Test path planning alone
ros2 run swarm_system path_planning_node --ros-args -p test_mode:=true

# Generate visualization
ros2 run swarm_system visualize_path --path_file /tmp/path_D1.json
```

---

## 📁 File Structure

```
swarm_ws/
├── src/
│   └── swarm_system/
│       ├── swarm_system/
│       │   ├── orchestrator.py
│       │   ├── scanning.py
│       │   ├── mapping.py
│       │   ├── poi.py
│       │   ├── task_allocation.py
│       │   ├── path_planning.py ⭐
│       │   ├── command_generation.py
│       │   ├── dgrc.py
│       │   └── px4_interface.py
│       ├── config/
│       │   ├── default.yaml
│       │   ├── path_planning.yaml
│       │   └── drones.yaml
│       ├── launch/
│       │   └── full_system.launch.py
│       ├── test/
│       ├── package.xml
│       └── setup.py
└── install/
```

---

## ⚙️ Key Configuration Parameters

### Path Planning (`config/path_planning.yaml`)
```yaml
path_planning:
  algorithm: "astar"
  grid_resolution: 0.5        # meters
  safety_margin: 1.0          # meters from obstacles
  smoothing_enabled: true
  timeout: 10.0               # seconds
  inter_drone_avoidance: false  # V1: Not implemented
```

### Scanning (`config/scanning.yaml`)
```yaml
scanning:
  default_altitude: 20.0      # meters
  default_speed: 3.0          # m/s
  line_spacing: 10.0          # meters
  overlap_percentage: 20      # %
```

### Task Allocation (`config/task_allocation.yaml`)
```yaml
task_allocation:
  algorithm: "greedy_nearest_neighbor"
  battery_reserve: 20.0       # % to keep
  max_tasks_per_drone: 10
```

---

## 🧪 Testing Checklist

### Unit Tests
- [ ] A* finds path in empty grid
- [ ] A* avoids static obstacles
- [ ] Task allocation respects battery constraints
- [ ] POI detection finds all obstacles
- [ ] Map building from sensor data

### Integration Tests
- [ ] Scanning → Mapping → POI pipeline
- [ ] POI → Allocation → Path Planning pipeline
- [ ] Path Planning → Command → DGRC → PX4

### System Tests
- [ ] Single drone, 4-waypoint mission (SITL)
- [ ] 50x50m area scan with mapping
- [ ] Multi-drone task allocation (3 drones)
- [ ] Low battery failsafe triggers RTL
- [ ] Path planning failure recovery

---

## 🎯 Performance Targets (V1)

| Metric | Target | Critical Path |
|--------|--------|---------------|
| Path planning | < 5s per path | YES |
| Mapping latency | < 2s for 100x100m | NO |
| Task allocation | < 1s for 10 POIs | NO |
| Telemetry rate | 10 Hz | YES |
| Command latency | < 200ms | YES |
| System memory | < 2GB | NO |
| Total planning | < 30s | YES |

---

## ⚠️ Known Limitations (V1)

1. **No inter-drone collision avoidance** - Paths planned independently
2. **Static obstacles only** - No dynamic replanning
3. **Sequential processing** - No parallel path planning
4. **Basic algorithms** - Not optimized for performance
5. **Limited error recovery** - Max 3 retries, then abort
6. **Single scan pattern** - Grid only (no spiral, adaptive)
7. **2D planning** - Altitude treated as constant per waypoint

---

## � Communication Pattern (V1 Simplified)

### Data Pipeline: Unidirectional →
```
Mapping → POI → Task Allocation → Path Planning → Commands → DGRC → PX4
```
- Pure forward data flow
- Each module reads from previous, writes to next
- No backward dependencies
- Simpler and faster

### Status Flow: To Orchestrator ↑
```
All Modules → Orchestrator (status, progress, errors)
Orchestrator → All Modules (start, pause, abort commands)
```

### Critical Bidirectional ↔
```
DGRC ↔ PX4: Commands down, telemetry up
Safety ↔ System: Monitor all, interrupt if needed
```

**Why this matters:**
- ✅ 40% less message traffic
- ✅ Easier to debug (clear data flow)
- ✅ No circular dependencies
- ✅ Faster implementation

---

## �🔄 State Machine Summary

```
[INIT]
  ↓ (all modules ready)
[READY]
  ↓ (start command)
[SCANNING] → collect sensor data
  ↓ (scan complete)
[MAPPING] → build occupancy grid
  ↓ (map ready)
[POI_DETECTION] → identify targets
  ↓ (POIs found)
[TASK_ALLOCATION] → assign to drones
  ↓ (assignments ready)
[PATH_PLANNING] → for each drone: plan path
  ↓ (all paths planned)
[COMMAND_GENERATION] → convert to flight commands
  ↓ (commands ready)
[EXECUTION] → drones fly missions
  ↓ (all missions complete)
[COMPLETED]

At any point:
  → [PAUSED] (pause command or low battery)
  → [ERROR] (module failure or timeout)
  → [RECOVERY] (attempt to resume)
```

---

## 📞 Support and Troubleshooting

### Common Issues

**Problem:** Path planning fails with "No path exists"
- **Solution:** Check if start/goal are in obstacle-free cells, increase safety margin, or provide intermediate waypoint

**Problem:** MAVROS not connecting to PX4
- **Solution:** Check serial port permissions (`sudo usermod -a -G dialout $USER`), verify baud rate (921600), restart MAVROS node

**Problem:** ROS2 nodes can't find each other
- **Solution:** Check `ROS_DOMAIN_ID` environment variable, verify network connectivity, disable firewall on loopback

**Problem:** Telemetry rate too low
- **Solution:** Check network latency, reduce message size, increase DDS buffer sizes

---

## 📚 Additional Resources

- **Full Specification:** See `VERSION_1_ARCHITECTURE.md` (2100+ lines)
- **Visual Flowchart:** Open `VERSION_1_FLOWCHART.drawio` in Draw.io
- **Meeting Notes:** Based on discussion on Dec 25, 2025
- **ROS2 Documentation:** https://docs.ros.org/en/humble/
- **PX4 Documentation:** https://docs.px4.io/
- **MAVROS:** https://github.com/mavlink/mavros

---

## 👥 Team Contacts and Roles

**Based on meeting discussion:**
- **Pankaj:** Team Lead / Technical Oversight
- **Shashi:** Architecture Design / Module Implementation
- **Tanishq:** Research / Documentation / Testing
- **Neha:** Knowledge Transfer / Training
- **Akash:** Task Coordination

**Development Approach (from meeting):**
1. Focus on ONE module at a time
2. Define inputs/outputs first (this document!)
3. Publish progress incrementally
4. Daily standups with quantified targets
5. Share MoM (Minutes of Meeting)

---

## 🏁 Version 1 Success Criteria

✅ **Functional:** Complete end-to-end mission execution  
✅ **Demonstrated:** Single drone scans 100x100m area  
✅ **Mapped:** Occupancy grid generated and visualized  
✅ **Detected:** At least 3 POIs identified  
✅ **Planned:** Collision-free paths for all drones  
✅ **Executed:** Drones fly autonomously to visit POIs  
✅ **Documented:** All modules have clear specs  
✅ **Tested:** Pass unit, integration, and SITL tests  

---

**Remember:** Version 1 is about making it WORK, not making it PERFECT!
Iterate and improve in future versions. 🚀
