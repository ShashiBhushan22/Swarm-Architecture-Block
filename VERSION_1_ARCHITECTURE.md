# Swarm Drone System - Version 1 Architecture (Detailed)
**Date:** December 25, 2025  
**Status:** Version 0.1 / Version 1 - Initial Functional Implementation  
**Goal:** End-to-end functional system with basic capabilities  
**Document Type:** Detailed Technical Specification

---

## Overview

This document provides a **detailed technical specification** for the Version 1 architecture of the drone swarm system. While the focus is on building something **functional end-to-end**, this version includes comprehensive details about:

- **Data structures and formats** for inputs/outputs
- **API specifications** for inter-module communication
- **State machines and workflows** for each module
- **Error handling and validation** approaches
- **Configuration parameters** and defaults
- **Implementation guidelines** and constraints
- **Integration points** and dependencies
- **Testing and validation** criteria

The goal is to provide enough detail for independent module development while maintaining the Version 1 principle of "make it work first, optimize later."

---

## System Architecture Flow (Optimized V1)

```
                    ┌─────────────────┐
                    │  ORCHESTRATOR   │  ← State Machine Manager
                    │                 │  ← Centralized Control
                    └────────┬────────┘
                             │ commands
                  status ←───┼───→ commands
                             ↓
                    ┌─────────────────┐
                    │  1. SCANNING    │  → /scanning/data
                    │     MODULE      │  Grid pattern, sensor data
                    └────────┬────────┘
                             │ data →
                             ↓
                    ┌─────────────────┐
                    │  2. MAPPING     │  → /mapping/occupancy_grid
                    │     MODULE      │  Occupancy grid, obstacles
                    └────────┬────────┘
                             │ data →
                             ↓
                    ┌─────────────────┐
                    │  3. POI         │  → /poi/poi_list
                    │    DETECTION    │  Rule-based identification
                    └────────┬────────┘
                             │ data →
                             ↓
                    ┌─────────────────┐
                    │  4. TASK        │  → /task_allocation/assignments
                    │    ALLOCATION   │  Greedy nearest-neighbor
                    └────────┬────────┘
                             │ data →
                             ↓
                    ┌─────────────────┐
                    │  5. PATH        │  → /path_planning/path/{id}
                    │    PLANNING     │  A* algorithm, for-loop
                    └────────┬────────┘
                             │ data →
                             ↓
                    ┌─────────────────┐
                    │  6. COMMAND     │  → /command_gen/commands/{id}
                    │    GENERATION   │  MAVLink commands
                    └────────┬────────┘
                             │ data →
                             ↓
                    ┌─────────────────┐
                    │  7. DGRC        │  ↔ Bidirectional
                    │     BRIDGE      │  ROS2 ↔ MAVLink (MAVROS)
                    └────────┬────────┘
                             │ cmds ↓ / telem ↑
                             ↓
                    ┌─────────────────┐
                    │  8. TELEMETRY   │  10-20 Hz telemetry
                    │     & PX4       │  Hardware interface
                    └─────────────────┘
```

**Communication Pattern (V1 Optimized):**
- **Data Pipeline (→):** Unidirectional forward flow - no backward dependencies
- **Status Flow (←):** All modules report status back to Orchestrator only
- **Commands (→):** Orchestrator broadcasts commands to specific modules as needed
- **Critical Bidirectional (↔):** DGRC ↔ PX4 only (must confirm command receipt)

---

## Detailed Module Specifications (Version 1)

---

### 1. ORCHESTRATOR MODULE

**Purpose:** Central coordinator that manages the overall workflow, module communication, state management, and system-level error handling.

#### 1.1 Inputs

**System Configuration:**
```python
{
    "mission_id": "string",           # Unique mission identifier
    "mission_type": "area_scan",      # Mission type (V1: area_scan only)
    "area_bounds": {
        "x_min": float,               # Meters
        "y_min": float,
        "x_max": float,
        "y_max": float,
        "z_min": float,               # Altitude min (default: 5m)

**Purpose:** Generate scan coverage plan and execute systematic area coverage to collect environmental data.

#### 2.1 Inputs

**Scan Request:**
```python
{
    "mission_id": "string",
    "scan_area": {
        "boundary_points": [           # Polygon vertices
            {"x": float, "y": float}
        ],
        "altitude": float,             # Scan altitude (meters)
        "no_fly_zones": [              # Optional obstacles
            {
                "boundary": [{"x": float, "y": float}],
                "altitude_min": float,
                "altitude_max": float
            }
        ]
    },
    "scan_parameters": {
        "pattern": "grid|spiral|adaptive",  # V1: grid only
        "line_spacing": float,         # Meters between scan lines
        "speed": float,                # m/s
        "sensor_config": {
            "camera_enabled": bool,
            "lidar_enabled": bool,
            "camera_angle": float,     # Degrees from nadir
            "overlap": float           # % (for photogrammetry)
        }
    },
    "assigned_drones": [
        {
            "drone_id": "string",
            "start_position": {"x": float, "y": float, "z": float}
        }
    ]
}
```

**Real-time Telemetry:**
```python
{
    "drone_id": "string",
    "position": {"x": float, "y": float, "z": float},
    "velocity": {"vx": float, "vy": float, "vz": float},
    "battery": float,                  # Percentage
    "sensor_status": {
        "camera": "active|inactive|error",
        "lidar": "active|inactive|error"
    },
    "timestamp": float
}
```

#### 2.2 Outputs

**Scan Plan:**
```python
{
    "mission_id": "string",
    "scan_waypoints": [
        {
            "drone_id": "string",
            "waypoints": [
                {
                    "id": int,
                    "position": {"x": float, "y": float, "z": float},
                    "yaw": float,          # Degrees
                    "capture_trigger": bool,  # Trigger sensor
                    "dwell_time": float    # Seconds to hover
                }
            ],
            "estimated_time": float,       # Minutes
            "estimated_battery": float     # % consumption
        }
    ],
    "total_area": float,               # Square meters
    "coverage_percentage": float       # Expected coverage
}
```

**Scan Data Output:**
```python
{
    "mission_id": "string",
    "drone_id": "string",
    "data_type": "image|pointcloud|combined",
    "data_points": [
        {
            "timestamp": float,
            "position": {"x": float, "y": float, "z": float},
            "data_file": "string",     # File path or URI
            "metadata": {
                "resolution": float,
                "quality": float,
                "sensor_id": "string"
            }
        }
    ],
    "coverage_achieved": float,        # Actual coverage %
    "status": "completed|partial|failed"
}
```

**Progress Updates:**
```python
{
    "mission_id": "string",
    "drone_id": "string",
    "current_waypoint": int,
    "waypoints_completed": int,
    "waypoints_total": int,
    "area_covered": float,             # Square meters
    "estimated_remaining_time": float, # Minutes
    "battery_remaining": float         # Percentage
}
```

#### 2.3 Scan Pattern Algorithm (V1: Grid Pattern)

**Grid Generation:**
```python
def generate_grid_pattern(area_bounds, line_spacing, altitude):
    """
    Generate parallel scan lines covering the area
    
    Steps:
    1. Calculate bounding box of scan area
    2. Determine scan line orientation (default: 0° - North-South)
    3. Generate parallel lines at line_spacing intervals
    4. Clip lines to area boundary
    5. Add turnaround points at line ends
    6. Optimize path to minimize turns
    """
    waypoints = []
    x_min, x_max = area_bounds['x_min'], area_bounds['x_max']
    y_min, y_max = area_bounds['y_min'], area_bounds['y_max']
    
    current_x = x_min
    direction = 1  # 1: forward (y_min→y_max), -1: backward
    
    while current_x <= x_max:
        if direction == 1:
            waypoints.append({'x': current_x, 'y': y_min, 'z': altitude})
            waypoints.append({'x': current_x, 'y': y_max, 'z': altitude})
        else:
            waypoints.append({'x': current_x, 'y': y_max, 'z': altitude})
            waypoints.append({'x': current_x, 'y': y_min, 'z': altitude})
        
        current_x += line_spacing
        direction *= -1
    
    return waypoints
```

#### 2.4 State Machine

**Purpose:** Process raw sensor data to build a structured map representation of the environment with obstacle identification and free space delineation.

#### 3.1 Inputs

**Sensor Data Stream:**
```python
{
    "mission_id": "string",
    "data_batch": [
        {
            "timestamp": float,
            "drone_id": "string",
            "position": {"x": float, "y": float, "z": float},
            "orientation": {"roll": float, "pitch": float, "yaw": float},
            "sensor_type": "camera|lidar|combined",
            "data": {
                "image_path": "string",        # For camera
                "point_cloud": [               # For lidar
                    {"x": float, "y": float, "z": float, "intensity": float}
                ],
                "depth_map": "string"          # Optional depth data
            }
        }
    ],
    "coordinate_frame": {
        "origin": {"x": float, "y": float, "z": float},
        "rotation": {"roll": float, "pitch": float, "yaw": float},
        "reference": "WGS84|local|custom"
    }
}
```

**Map Configuration:**
```python
{
    "map_bounds": {
        "x_min": float, "x_max": float,
        "y_min": float, "y_max": float,
        "z_min": float, "z_max": float
    },
    "resolution": float,               # Cell size in meters
    "map_type": "2D|2.5D|3D",         # V1: 2D only
    "update_mode": "incremental|batch"  # V1: batch only
}
```

#### 3.2 Outputs

**Occupancy Grid Map:**
```python
{
    "mission_id": "string",
    "map_metadata": {
        "resolution": float,           # meters per cell
        "width": int,                  # cells
        "height": int,                 # cells
        "origin": {"x": float, "y": float, "z": float},
        "timestamp": float,
        "frame_id": "string"
    },
    "occupancy_grid": {
        "data": [int],                 # Flattened 1D array
                                       # 0: free, 100: occupied, -1: unknown
        "encoding": "row_major",
        "shape": [int, int]            # [height, width]
    },
    "metadata": {
        "total_cells": int,
        "occupied_cells": int,
        "free_cells": int,
        "unknown_cells": int,
        "coverage_percentage": float
    }
}
```

**Obstacle List:**
```python
{
    "obstacles": [
        {
            "id": int,
            "type": "static|dynamic|unknown",  # V1: static only
            "geometry": "point|line|polygon|circle",
            "coordinates": [
                {"x": float, "y": float}
            ],
            "properties": {
                "height": float,       # meters
                "confidence": float,   # 0-1
                "source": "sensor_type"
            }
        }
    ],
    "timestamp": float
}
```

**Map Visualization Data:**
```python
{
    "map_image": "string",             # Path to visualization image
    "legend": {
        "free": {"color": "#FFFFFF", "value": 0},
        "occupied": {"color": "#000000", "value": 100},
        "unknown": {"color": "#808080", "value": -1}
    },
    "markers": [                       # For RViz/plotting
        {
            "type": "obstacle|poi|drone",
            "position": {"x": float, "y": float},
            "color": "string",
            "label": "string"
        }
    ]
}
```

#### 3.3 Mapping Algorithm (V1: Simple Occupancy Grid)

**Occupancy Grid Generation:**
```python
def build_occupancy_grid(sensor_data, resolution, bounds):
    """
    Build 2D occupancy grid from sensor data
    
    Steps:
    1. Initialize grid with all cells as unknown (-1)
    2. For each sensor reading:
       a. Transform point to map frame
       b. Apply ray-casting from sensor position to detected point
       c. Mark free cells along the ray
       d. Mark occupied cell at detection point
    3. Apply median filter to reduce noise
    4. Calculate statistics
    """
    
    # Calculate grid dimensions
    width = int((bounds['x_max'] - bounds['x_min']) / resolution)
    height = int((bounds['y_max'] - bounds['y_min']) / resolution)
    
    # Initialize grid
    grid = np.full((height, width), -1, dtype=np.int8)
    
    for reading in sensor_data:
        sensor_pos = reading['position']
        
        if reading['sensor_type'] == 'lidar':
            for point in reading['data']['point_cloud']:
                # Ray casting
                ray_cells = bresenham_line(
                    world_to_grid(sensor_pos, bounds, resolution),
                    world_to_grid(point, bounds, resolution)
                )
                
                # Mark free cells (along ray)
                for cell in ray_cells[:-1]:
                    if grid[cell[1], cell[0]] == -1:
                        grid[cell[1], cell[0]] = 0
                
                # Mark occupied cell (at endpoint)
                endpoint = ray_cells[-1]
                grid[endpoint[1], endpoint[0]] = 100
    
    return grid
```

**Obstacle Extraction:**
```python
def extract_obstacles(occupancy_grid, min_size=5):
    """
    Extract discrete obstacles from occupancy grid
    
    Steps:
    1. Threshold occupied cells (>= 50)
    2. Apply connected components analysis
    3. Filter components by minimum size
    4. Compute bounding boxes or convex hulls
    5. Calculate properties (centroid, size, orientation)
    """
    
    obstacles = []
    
    # Threshold and find connected components
    binary_map = (occupancy_grid >= 50).astype(np.uint8)
    num_labels, labels = cv2.connectedComponents(binary_map)
    
    for label_id in range(1, num_labels):
        component = (labels == label_id)
        area = np.sum(component)
        
        if area >= min_size:
            # Extract obstacle properties
            points = np.argwhere(component)
            centroid = np.mean(points, axis=0)
            
            # Simplified polygon (bounding box for V1)
            min_pt = np.min(points, axis=0)
            max_pt = np.max(points, axis=0)
            
            obstacles.append({
                'id': label_id,
                'type': 'static',
                'geometry': 'polygon',
                'coordinates': [
                    grid_to_world(min_pt, ...),
                    grid_to_world([min_pt[0], max_pt[1]], ...),
                    grid_to_world(max_pt, ...),
                    grid_to_world([max_pt[0], min_pt[1]], ...)
                ],
                'properties': {
                    'height': 2.0,  # Default assumption for V1
                    'confidence': 0.8,
                    'source': 'lidar'
                }
            })
    
    return obstacles
```

#### 3.4 State Machine

```
[IDLE] → [RECEIVING_DATA] → [PROCESSING] → [PUBLISHING] → [IDLE]
                                  ↓
                            [ERROR] → [RECOVERY]
```

#### 3.5 Implementation Details

**ROS2 Interface:**
```yaml
Publishers:
  - /mapping/occupancy_grid: OccupancyGrid (nav_msgs)
  - /mapping/obstacles: ObstacleArray (custom msg)
  - /mapping/visualization: MarkerArray (visualization_msgs)
  - /mapping/status: ModuleStatus

Subscribers:
  - /scan/data: SensorData
  - /orchestrator/command: OrchestratorCommand

Services:
  - /mapping/build_map: BuildMap.srv
  - /mapping/get_map: GetMap.srv
  - /mapping/clear_map: std_srvs/Empty
```

**Configuration:**
```yaml
mapping:
  grid_resolution: 0.5          # meters per cell
  unknown_threshold: -1
  free_threshold: 25            # < 25 = free
  occupied_threshold: 65        # >= 65 = occupied
  min_obstacle_size: 5          # cells
  filter_kernel_size: 3         # for median filter
  visualization_enabled: true
  map_frame: "map"
  sensor_frame: "lidar_link"
  update_rate: 1.0              # Hz for batch updates
```

**Data Structures:**
```python
# Internal representation
class OccupancyGrid:
    def __init__(self, resolution, bounds):
        self.resolution = resolution
        self.bounds = bounds
        self.grid = np.full((height, width), -1, dtype=np.int8)
        self.metadata = {}
    
    def update_cell(self, x, y, value):
        pass
    
    def get_cell(self, x, y):
        pass
    
    def world_to_grid(self, world_pos):
        pass
    
    def grid_to_world(self, grid_pos):
        pass
```

#### 3.6 Error Handling

| Error Condition | Detection | Recovery |
|----------------|-----------|----------|
| Invalid sensor data | Data validation | Skip reading, log warning |
| Out of bounds data | Coordinate check | Clip to map bounds |
| Memory overflow | Memory monitoring | Reduce resolution, warn user |
| Corrupted data file | File checksum | Request re-scan of area |
| Coordinate frame mismatch | TF lookup failure | Use default transform, flag error |

#### 3.7 Performance Considerations

**Memory Management:**
- Grid size limited to 10,000 x 10,000 cells (100MB)
- Use sparse representation if >80% cells are unknown
- Periodic memory cleanup for old data

**Processing Optimization:**
- Batch process sensor data every 1 second
- Use numpy vectorization for grid operations
- Multi-threaded obstacle extraction

#### 3.8 Testing Criteria

- ✓ Build occupancy grid from simulated lidar data
- ✓ Detect and extract 5+ distinct obstacles
- ✓ Publish map at 1Hz with <100ms latency
- ✓ Handle 1000+ sensor readings per second
- ✓ Correctly transform between world and grid coordinatesn message
  - /scan/progress: ScanProgress message
  - /scan/data: SensorData message
  - /scan/status: ModuleStatus message

Subscribers:
  - /orchestrator/command: OrchestratorCommand
  - /drone/telemetry: DroneTelemetry
  - /drone/sensor_data: SensorData

Services:
  - /scan/generate_plan: GenerateScanPlan.srv
  - /scan/execute: ExecuteScan.srv
```

**Configuration Parameters:**
```yaml
scanning:
  default_altitude: 20.0        # meters
  default_speed: 3.0            # m/s
  line_spacing: 10.0            # meters
  overlap_percentage: 20        # %
  waypoint_tolerance: 0.5       # meters
  image_capture_interval: 2.0   # seconds
  data_storage_path: "/data/scans/"
```

#### 2.6 Error Handling

| Error Condition | Detection | Recovery |
|----------------|-----------|----------|
| GPS signal lost | No position update >1s | Hover and wait (max 30s) |
| Sensor failure | Status = error | Continue scan, log failure |
| Battery < 30% | Telemetry monitoring | Return to home, save partial data |
| Wind > threshold | IMU data | Reduce speed, increase altitude |
| Obstacle detected | Collision avoidance | Re-plan path around obstacle |

#### 2.7 Testing Criteria

- ✓ Generate valid grid pattern for rectangular area
- ✓ Execute scan with single drone
- ✓ Collect and store sensor data
- ✓ Calculate actual coverage percentage
- ✓ Handle scan interruption and resume
        }
    ],
    "scan_parameters": {
        "grid_resolution": float,      # meters (default: 1.0)
        "overlap_percentage": float    # % (default: 20)
    }
}
```

**Runtime Commands:**
```python
{
    "command": "start|pause|resume|abort|status",
    "timestamp": float,               # Unix timestamp
    "parameters": dict                # Command-specific params
}
```

**Module Status Updates:**
```python
{
    "module_name": "string",
    "status": "idle|running|completed|error",
    "progress": float,                # 0.0 to 1.0
    "message": "string",
    "timestamp": float,
    "error_details": dict             # If status == error
}
```

#### 1.2 Outputs

**Module Commands:**
```python
{
    "target_module": "string",        # Module name
    "command": "initialize|execute|pause|terminate",
    "payload": dict,                  # Module-specific data
    "request_id": "string",           # Unique request ID
    "timestamp": float
}
```

**System Status Broadcast:**
```python
{
    "system_state": "initializing|ready|executing|paused|completed|error",
    "active_module": "string",
    "mission_progress": float,        # Overall % complete
    "drones_status": [
        {
            "drone_id": "string",
            "state": "string",
            "battery": float,         # Percentage
            "position": {"x": float, "y": float, "z": float}
        }
    ],
    "timestamp": float
}
```

#### 1.3 State Machine (SMACH Implementation)

**SMACH Reference:** http://wiki.ros.org/smach | https://github.com/ros/executive_smach

**Hierarchical State Machine Structure:**

```python
ORCHESTRATOR_SM (Top-level StateMachine)
│
├── INIT
├── READY
├── MISSION_EXECUTION (Nested StateMachine)
│   ├── SCANNING
│   ├── MAPPING
│   ├── POI_DETECTION
│   ├── TASK_ALLOCATION
│   ├── PATH_PLANNING
│   ├── COMMAND_GENERATION
│   └── DRONE_EXECUTION (Concurrence Container)
│       ├── Monitor Telemetry
│       ├── Monitor Safety
│       └── Track Progress
├── PAUSED
├── ERROR_RECOVERY (Nested StateMachine)
│   ├── DIAGNOSE
│   ├── RETRY
│   └── FALLBACK
├── COMPLETED
└── ABORTED
```

**State Definitions & Outcomes:**

```python
from smach import StateMachine, State, Concurrence
from smach_ros import ServiceState, MonitorState

# Top-level outcomes
ORCHESTRATOR_OUTCOMES = ['mission_complete', 'mission_aborted']

# State outcomes mapping
STATE_OUTCOMES = {
    'INIT': ['initialized', 'init_failed'],
    'READY': ['start_mission', 'shutdown'],
    'SCANNING': ['scan_complete', 'scan_failed', 'paused'],
    'MAPPING': ['map_ready', 'map_failed', 'paused'],
    'POI_DETECTION': ['pois_detected', 'no_pois', 'detection_failed', 'paused'],
    'TASK_ALLOCATION': ['tasks_allocated', 'allocation_failed', 'paused'],
    'PATH_PLANNING': ['paths_ready', 'planning_failed', 'paused'],
    'COMMAND_GENERATION': ['commands_ready', 'cmd_gen_failed', 'paused'],
    'DRONE_EXECUTION': ['execution_complete', 'execution_failed', 'paused'],
    'PAUSED': ['resume', 'abort'],
    'ERROR_RECOVERY': ['recovered', 'recovery_failed'],
    'COMPLETED': [],
    'ABORTED': []
}
```

**Complete SMACH State Machine Definition:**

```python
#!/usr/bin/env python3
"""
Orchestrator SMACH State Machine
Version 1 - Sequential Mission Execution with Error Recovery
"""

import rospy
import smach
import smach_ros
from std_msgs.msg import String
from swarm_msgs.srv import StartMission, ModuleCommand

class OrchestratorStateMachine:
    def __init__(self):
        # Create top-level state machine
        self.sm = smach.StateMachine(
            outcomes=['mission_complete', 'mission_aborted']
        )
        
        # Shared user data across states
        self.sm.userdata.mission_id = ''
        self.sm.userdata.scan_area = {}
        self.sm.userdata.occupancy_grid = None
        self.sm.userdata.poi_list = []
        self.sm.userdata.task_assignments = {}
        self.sm.userdata.drone_paths = {}
        self.sm.userdata.error_count = 0
        self.sm.userdata.current_module = ''
        
        # Build state machine
        self._build_state_machine()
    
    def _build_state_machine(self):
        with self.sm:
            # ============ INITIALIZATION STATE ============
            smach.StateMachine.add(
                'INIT',
                InitializeSystem(),
                transitions={
                    'initialized': 'READY',
                    'init_failed': 'ABORTED'
                },
                remapping={
                    'mission_config': 'mission_id'
                }
            )
            
            # ============ READY STATE ============
            smach.StateMachine.add(
                'READY',
                ReadyState(),
                transitions={
                    'start_mission': 'MISSION_EXECUTION',
                    'shutdown': 'mission_complete'
                }
            )
            
            # ============ MISSION EXECUTION (Nested) ============
            mission_sm = self._create_mission_execution_sm()
            smach.StateMachine.add(
                'MISSION_EXECUTION',
                mission_sm,
                transitions={
                    'mission_success': 'COMPLETED',
                    'mission_failed': 'ERROR_RECOVERY',
                    'mission_paused': 'PAUSED'
                }
            )
            
            # ============ PAUSED STATE ============
            smach.StateMachine.add(
                'PAUSED',
                PausedState(),
                transitions={
                    'resume': 'MISSION_EXECUTION',
                    'abort': 'ABORTED'
                }
            )
            
            # ============ ERROR RECOVERY (Nested) ============
            recovery_sm = self._create_error_recovery_sm()
            smach.StateMachine.add(
                'ERROR_RECOVERY',
                recovery_sm,
                transitions={
                    'recovered': 'MISSION_EXECUTION',
                    'recovery_failed': 'ABORTED'
                }
            )
            
            # ============ COMPLETED STATE ============
            smach.StateMachine.add(
                'COMPLETED',
                CompletedState(),
                transitions={
                    'done': 'mission_complete'
                }
            )
            
            # ============ ABORTED STATE ============
            smach.StateMachine.add(
                'ABORTED',
                AbortedState(),
                transitions={
                    'done': 'mission_aborted'
                }
            )
    
    def _create_mission_execution_sm(self):
        """Create nested state machine for mission execution pipeline"""
        mission_sm = smach.StateMachine(
            outcomes=['mission_success', 'mission_failed', 'mission_paused']
        )
        mission_sm.userdata.scan_data = None
        mission_sm.userdata.map_data = None
        
        with mission_sm:
            # State 1: SCANNING
            smach.StateMachine.add(
                'SCANNING',
                ScanningState(),
                transitions={
                    'scan_complete': 'MAPPING',
                    'scan_failed': 'mission_failed',
                    'paused': 'mission_paused'
                },
                remapping={
                    'scan_area': 'scan_area',
                    'scan_data': 'scan_data'
                }
            )
            
            # State 2: MAPPING
            smach.StateMachine.add(
                'MAPPING',
                MappingState(),
                transitions={
                    'map_ready': 'POI_DETECTION',
                    'map_failed': 'mission_failed',
                    'paused': 'mission_paused'
                },
                remapping={
                    'scan_data': 'scan_data',
                    'occupancy_grid': 'occupancy_grid'
                }
            )
            
            # State 3: POI DETECTION
            smach.StateMachine.add(
                'POI_DETECTION',
                POIDetectionState(),
                transitions={
                    'pois_detected': 'TASK_ALLOCATION',
                    'no_pois': 'mission_success',  # Valid completion
                    'detection_failed': 'mission_failed',
                    'paused': 'mission_paused'
                },
                remapping={
                    'occupancy_grid': 'occupancy_grid',
                    'poi_list': 'poi_list'
                }
            )
            
            # State 4: TASK ALLOCATION
            smach.StateMachine.add(
                'TASK_ALLOCATION',
                TaskAllocationState(),
                transitions={
                    'tasks_allocated': 'PATH_PLANNING',
                    'allocation_failed': 'mission_failed',
                    'paused': 'mission_paused'
                },
                remapping={
                    'poi_list': 'poi_list',
                    'task_assignments': 'task_assignments'
                }
            )
            
            # State 5: PATH PLANNING
            smach.StateMachine.add(
                'PATH_PLANNING',
                PathPlanningState(),
                transitions={
                    'paths_ready': 'COMMAND_GENERATION',
                    'planning_failed': 'mission_failed',
                    'paused': 'mission_paused'
                },
                remapping={
                    'task_assignments': 'task_assignments',
                    'occupancy_grid': 'occupancy_grid',
                    'drone_paths': 'drone_paths'
                }
            )
            
            # State 6: COMMAND GENERATION
            smach.StateMachine.add(
                'COMMAND_GENERATION',
                CommandGenerationState(),
                transitions={
                    'commands_ready': 'DRONE_EXECUTION',
                    'cmd_gen_failed': 'mission_failed',
                    'paused': 'mission_paused'
                },
                remapping={
                    'drone_paths': 'drone_paths',
                    'drone_commands': 'drone_commands'
                }
            )
            
            # State 7: DRONE EXECUTION (Concurrent monitoring)
            execution_cc = self._create_execution_concurrence()
            smach.StateMachine.add(
                'DRONE_EXECUTION',
                execution_cc,
                transitions={
                    'execution_complete': 'mission_success',
                    'execution_failed': 'mission_failed',
                    'paused': 'mission_paused'
                }
            )
        
        return mission_sm
    
    def _create_execution_concurrence(self):
        """Create concurrent container for drone execution monitoring"""
        execution_cc = smach.Concurrence(
            outcomes=['execution_complete', 'execution_failed', 'paused'],
            default_outcome='execution_failed',
            outcome_map={
                'execution_complete': {
                    'EXECUTE_MISSION': 'success',
                    'MONITOR_TELEMETRY': 'nominal',
                    'MONITOR_SAFETY': 'safe'
                },
                'execution_failed': {
                    'MONITOR_SAFETY': 'unsafe'
                },
                'paused': {
                    'MONITOR_TELEMETRY': 'pause_requested'
                }
            },
            child_termination_cb=self._execution_term_cb
        )
        
        with execution_cc:
            # Primary execution thread
            smach.Concurrence.add(
                'EXECUTE_MISSION',
                ExecuteMissionState()
            )
            
            # Telemetry monitoring thread
            smach.Concurrence.add(
                'MONITOR_TELEMETRY',
                MonitorTelemetryState()
            )
            
            # Safety monitoring thread
            smach.Concurrence.add(
                'MONITOR_SAFETY',
                MonitorSafetyState()
            )
        
        return execution_cc
    
    def _execution_term_cb(self, outcome_map):
        """Callback for concurrent execution termination logic"""
        # If safety monitor reports unsafe, terminate all
        if outcome_map['MONITOR_SAFETY'] == 'unsafe':
            return True
        # If telemetry reports pause, terminate all
        if outcome_map['MONITOR_TELEMETRY'] == 'pause_requested':
            return True
        # If mission execution completes, terminate all
        if outcome_map['EXECUTE_MISSION'] == 'success':
            return True
        return False
    
    def _create_error_recovery_sm(self):
        """Create nested state machine for error recovery"""
        recovery_sm = smach.StateMachine(
            outcomes=['recovered', 'recovery_failed']
        )
        
        with recovery_sm:
            # Diagnose error
            smach.StateMachine.add(
                'DIAGNOSE',
                DiagnoseErrorState(),
                transitions={
                    'retry_possible': 'RETRY',
                    'use_fallback': 'FALLBACK',
                    'unrecoverable': 'recovery_failed'
                }
            )
            
            # Attempt retry
            smach.StateMachine.add(
                'RETRY',
                RetryState(),
                transitions={
                    'retry_success': 'recovered',
                    'retry_failed': 'FALLBACK'
                }
            )
            
            # Fallback strategy
            smach.StateMachine.add(
                'FALLBACK',
                FallbackState(),
                transitions={
                    'fallback_success': 'recovered',
                    'fallback_failed': 'recovery_failed'
                }
            )
        
        return recovery_sm


# ============ INDIVIDUAL STATE IMPLEMENTATIONS ============

class InitializeSystem(smach.State):
    """Initialize all modules and check system readiness"""
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['initialized', 'init_failed'],
            input_keys=['mission_config'],
            output_keys=['mission_id']
        )
        self.init_timeout = 30.0  # seconds
    
    def execute(self, userdata):
        rospy.loginfo('ORCHESTRATOR: Initializing system...')
        
        # Check all module connections
        modules = ['scanning', 'mapping', 'poi', 'task_allocation', 
                   'path_planning', 'command_gen', 'dgrc', 'telemetry']
        
        for module in modules:
            try:
                rospy.wait_for_service(f'/{module}/status', timeout=5.0)
                rospy.loginfo(f'Module {module} ready')
            except rospy.ROSException:
                rospy.logerr(f'Module {module} not responding')
                return 'init_failed'
        
        # Generate mission ID
        userdata.mission_id = f"mission_{rospy.Time.now().secs}"
        rospy.loginfo(f'System initialized: {userdata.mission_id}')
        return 'initialized'


class ReadyState(smach.State):
    """Wait for mission start command"""
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['start_mission', 'shutdown']
        )
        self.start_sub = rospy.Subscriber(
            '/orchestrator/start', String, self._start_cb
        )
        self.mission_started = False
    
    def _start_cb(self, msg):
        self.mission_started = True
    
    def execute(self, userdata):
        rospy.loginfo('ORCHESTRATOR: Ready - waiting for start command')
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            if self.mission_started:
                self.mission_started = False
                return 'start_mission'
            rate.sleep()
        
        return 'shutdown'


# Additional state classes: ScanningState, MappingState, POIDetectionState,
# TaskAllocationState, PathPlanningState, CommandGenerationState,
# ExecuteMissionState, MonitorTelemetryState, MonitorSafetyState,
# DiagnoseErrorState, RetryState, FallbackState, CompletedState, AbortedState
# (Follow similar pattern with service calls and state transitions)


# ============ MAIN EXECUTION ============

def main():
    rospy.init_node('orchestrator_smach')
    
    # Create orchestrator state machine
    orchestrator = OrchestratorStateMachine()
    
    # Publish state changes to topic for monitoring
    state_pub = rospy.Publisher('/orchestrator/state', String, queue_size=10)
    
    # Register state change callback
    def state_change_cb(userdata, active_states):
        state_pub.publish(String(data=str(active_states)))
        rospy.loginfo(f'State: {active_states}')
    
    orchestrator.sm.register_transition_cb(state_change_cb)
    
    # Execute state machine
    outcome = orchestrator.sm.execute()
    
    rospy.loginfo(f'Final outcome: {outcome}')


if __name__ == '__main__':
    main()
```

**State Transition Diagram (SMACH):**

```
                    ┌──────────┐
                    │   INIT   │
                    └────┬─────┘
                         │ initialized
                         ↓
                    ┌──────────┐
                    │  READY   │←───────────────┐
                    └────┬─────┘                │
                         │ start_mission        │
                         ↓                      │
         ┌───────────────────────────────┐     │
         │   MISSION_EXECUTION (Nested)  │     │
         │                               │     │
         │  SCANNING → MAPPING →         │     │
         │  POI_DETECTION →              │     │
         │  TASK_ALLOCATION →            │     │
         │  PATH_PLANNING →              │     │
         │  COMMAND_GENERATION →         │     │
         │  DRONE_EXECUTION              │     │
         │     (Concurrent)              │     │
         └───┬───────────┬───────┬───────┘     │
             │           │       │             │
  mission_  │           │       │ mission_    │
  success   │           │       │ paused      │
             │           │       │             │
             ↓           │       ↓             │
        ┌──────┐         │  ┌────────┐        │
        │COMPLT│         │  │ PAUSED │────────┘
        │  ED  │         │  └────┬───┘ resume
        └──────┘         │       │
                         │       │ abort
                         │       ↓
                         │  ┌────────┐
           mission_      │  │ABORTED │
           failed        │  └────────┘
                         │
                         ↓
         ┌───────────────────────────┐
         │  ERROR_RECOVERY (Nested)  │
         │                           │
         │  DIAGNOSE → RETRY →       │
         │  FALLBACK                 │
         └───┬───────────────┬───────┘
             │               │
    recovered│               │ recovery_failed
             │               │
             └───────┐       │
                     │       │
                     ↓       ↓
                 [Back to    ABORTED
                  Mission]
```

**Key SMACH Features Used:**

1. **Hierarchical State Machines:** Nested for mission execution and error recovery
2. **Concurrence Container:** Parallel execution monitoring during drone flight
3. **User Data:** Shared data between states (mission_id, maps, paths, assignments)
4. **Service Integration:** ROS service calls within states
5. **Introspection Server:** Real-time visualization at `localhost:5555`
6. **Preemption:** Can pause/abort from any state
7. **Outcome Mapping:** Define complex transition logic for concurrent states

**Installation & Dependencies:**

```bash
# Install SMACH for ROS2 Humble
sudo apt install ros-humble-executive-smach

# Note: smach-viewer is ROS1 only, not available for ROS2 yet
# We'll use alternative visualization methods (see below)
```

**Running the Orchestrator:**
```bash
# Run orchestrator
ros2 run swarm_orchestrator orchestrator_smach.py

# Monitor state transitions via logging
ros2 topic echo /orchestrator/state

# Monitor in separate terminal
ros2 topic echo /orchestrator/status
```

**Alternative Visualization Options for ROS2:**

**Option 1: Custom Web Dashboard (Recommended for V1)**
```bash
# Create simple state monitor
ros2 run swarm_orchestrator state_monitor_gui.py
# Opens simple GUI showing current state, transitions, and userdata
```

**Option 2: Command Line Monitor**
```bash
# Watch state changes in real-time
watch -n 0.5 'ros2 topic echo --once /orchestrator/state'

# Or use rqt
ros2 run rqt_gui rqt_gui
# Add Topic Monitor plugin → /orchestrator/state
```

**Option 3: Logging**
```bash
# View detailed logs
ros2 run swarm_orchestrator orchestrator_smach.py --ros-args --log-level debug

# Or check logs after
ros2 run rqt_console rqt_console
```

**Option 4: PlotJuggler (Best for Real-time Monitoring)**
```bash
# Install
sudo apt install ros-humble-plotjuggler-ros

# Run and add state topics
ros2 run plotjuggler plotjuggler
# Add streaming: ROS2 Topics
# Subscribe to /orchestrator/state, /orchestrator/status
```

**SMACH Advantages for V1:**

✅ **Visual Debugging**: See current state in real-time  
✅ **Hierarchical Organization**: Nested state machines for complex workflows  
✅ **Concurrent Execution**: Monitor telemetry/safety while mission runs  
✅ **Error Recovery**: Structured approach to handling failures  
✅ **User Data Flow**: Explicit data passing between states  
✅ **ROS Integration**: Built-in support for topics/services  
✅ **Preemption**: Graceful pause/abort capabilities  

**Reference:**  
📘 **SMACH Tutorials**: http://wiki.ros.org/smach/Tutorials  
📄 **Böhren, J., et al. (2011)** "Towards Autonomous Robotic Butlers: Lessons Learned with the PR2", IEEE ICRA

#### 1.4 Implementation Details

**Version 1 Approach:**
- Single-threaded sequential execution
- Blocking wait for module completion
- 30-second timeout per module operation
- Maximum 3 retry attempts on module failure
- Simple logging to console and file

**Communication Protocol:**
- ROS2 Topics for async messages
- ROS2 Services for sync request-response
- 10Hz status update frequency

**Configuration File (YAML):**
```yaml
orchestrator:
  timeout_seconds: 30
  max_retries: 3
  heartbeat_interval_ms: 100
  log_level: INFO
  recovery_enabled: true
```

#### 1.5 Error Handling

| Error Type | Recovery Action | Max Retries |
|------------|----------------|-------------|
| Module timeout | Restart module | 3 |
| Invalid input | Request correction | 1 |
| Drone disconnected | Wait and retry | 5 |
| Battery critical | Initiate landing | 0 (immediate) |
| Path planning failed | Re-request with relaxed constraints | 2 |

#### 1.6 Testing Criteria

- ✓ Successfully coordinates all 8 modules
- ✓ Handles single drone mission end-to-end
- ✓ Recovers from single module failure
- ✓ Generates complete execution log
- ✓ Updates system status at 10Hz

---

### 2. SCANNING MODULE
**Purpose:** Drone scans the designated area to collect environmental data.

**Inputs:**
- Scan area boundaries (x_min, y_min, x_max, y_max)
- Drone ID(s)
- Current drone position(s)

**Outputs:**
- Raw scan data (point cloud, images, sensor readings)
- Scan completion status

**Version 1 Implementation:**
- Basic grid/raster scanning pattern
- Single drone scanning initially
- Simple sensor data collection

---

### 3. MAPPING MODULE
**Purpose:** Process scan data and build a map representation of the environment.

**Inputs:**
- Raw scan data from Scanning Module
- Coordinate system reference

**Outputs:**
- 2D/3D occupancy grid map
- Obstacle locations
- Free space identification

**Version 1 Implementation:**
- Simple occupancy grid
- Basic obstacle detection
- Fixed resolution grid

---

### 4. POINTS OF INTEREST (POI) MODULE
**Purpose:** Identify and list areas/objects that require drone attention.

**Inputs:**
- Map from Mapping Module
- POI detection criteria (e.g., specific objects, anomalies)

**Outputs:**
- List of POI coordinates [(x1, y1), (x2, y2), ...]
- POI priorities (if applicable)

**Version 1 Implementation:**
- Simple rule-based POI detection
- Basic coordinate list
- No complex classification initially

---

### 5. TASK ALLOCATION MODULE
**Purpose:** Assign POIs to available drones efficiently.

**Inputs:**
- List of POIs from POI Module
- Number of available drones
- Current drone positions

**Outputs:**
- Assignment mapping: {Drone_ID: [assigned_POIs]}
- Task sequence for each drone

**Version 1 Implementation:**
- Simple round-robin or nearest-neighbor assignment
- No complex optimization initially
- Basic load balancing

---

### 6. PATH PLANNING MODULE
**Purpose:** Generate collision-free paths for each drone to visit assigned POIs.

**Inputs:**
- Drone ID
- Start position: (x_start, y_start)
- Goal position: (x_goal, y_goal)
- Map/occupancy grid from Mapping Module

**Input Format (per request):**
```python
{
    "drone_id": "D1",
    "start": (x1, y1),
    "goal": (x_end, y_end),
    "map": occupancy_grid
}
```

**Outputs:**
- Path waypoints: [(x1, y1), (x2, y2), ..., (x_goal, y_goal)]
- Path length/cost
- Path validity status

**Output Format:**
```python
{
    "drone_id": "D1",
    "path": [(x1, y1), (x2, y2), ..., (x_n, y_n)],
    "status": "success/failed",
    "cost": path_length
}
```

**Version 1 Implementation:**
- **Basic A* algorithm** for pathfinding
- **No inter-drone collision avoidance** initially
- **For-loop processing:** Process all path planning requests sequentially
- **Immediate publishing:** Publish path as soon as it's generated for each drone
- Simple 2D grid-based planning
- Static obstacles only

**Processing Logic:**
```python
for drone_request in all_path_requests:
    path = run_A_star(drone_request)
    publish_path(path)  # Immediate publication
```

---

### 7. COMMAND GENERATION MODULE

**Purpose:** Translate planned paths into flight controller-compatible command sequences with timing, velocity profiles, and action triggers.

#### 7.1 Inputs

**Planned Path:**
```python
{
    "drone_id": "string",
    "path": [
        {
            "position": {"x": float, "y": float, "z": float},
            "yaw": float,
            "velocity": {"vx": float, "vy": float, "vz": float},
            "timestamp": float
        }
    ],
    "path_metadata": dict
}
```

**Drone Specifications:**
```python
{
    "drone_id": "string",
    "flight_modes": ["OFFBOARD", "AUTO.MISSION", "POSITION"],
    "max_velocity": {"horizontal": float, "vertical": float},
    "max_acceleration": float,
    "response_time": float  # Latency in seconds
}
```

#### 7.2 Outputs

**Command Sequence:**
```python
{
    "drone_id": "string",
    "command_sequence": [
        {
            "command_id": int,
            "command_type": "TAKEOFF|WAYPOINT|LAND|ACTION|RETURN_HOME",
            "parameters": {
                "position": {"x": float, "y": float, "z": float},
                "velocity": float,
                "yaw": float,
                "hold_time": float,
                "acceptance_radius": float
            },
            "actions": [  # Triggered at this waypoint
                {
                    "action_type": "CAPTURE_IMAGE|START_VIDEO|TRIGGER_SENSOR",
                    "parameters": dict
                }
            ],
            "timing": {
                "eta": float,  # Expected time of arrival
                "execution_window": float  # Max time to execute
            }
        }
    ],
    "mission_metadata": {
        "total_waypoints": int,
        "estimated_duration": float,
        "estimated_battery_usage": float
    }
}
```

**MAVLink Mission Items:**
```python
{
    "mission_items": [
        {
            "seq": int,
            "frame": int,  # MAV_FRAME
            "command": int,  # MAV_CMD
            "param1": float,
            "param2": float,
            "param3": float,
            "param4": float,
            "x": float,  # Latitude or local x
            "y": float,  # Longitude or local y
            "z": float,  # Altitude or local z
            "autocontinue": int
        }
    ]
}
```

#### 7.3 Command Generation Algorithm

```python
def generate_commands(path, drone_spec):
    """
    Convert path to command sequence
    
    Steps:
    1. Add takeoff command (if not already airborne)
    2. Convert each path waypoint to flight command
    3. Add sensor trigger actions at POIs
    4. Calculate velocity and timing
    5. Add landing/return command at end
    6. Validate command sequence
    """
    
    commands = []
    
    # Command 1: Takeoff
    if not is_airborne(drone_id):
        commands.append({
            'command_id': 0,
            'command_type': 'TAKEOFF',
            'parameters': {
                'altitude': path[0]['position']['z'],
                'yaw': 0.0
            }
        })
    
    # Commands 2-N: Waypoints
    for i, waypoint in enumerate(path):
        cmd = {
            'command_id': len(commands),
            'command_type': 'WAYPOINT',
            'parameters': {
                'position': waypoint['position'],
                'velocity': calculate_safe_velocity(waypoint, drone_spec),
                'yaw': waypoint.get('yaw', 0.0),
                'acceptance_radius': 1.0,  # meters
                'hold_time': 0.0
            },
            'actions': []
        }
        
        # Add sensor actions if this is a POI
        if waypoint.get('is_poi', False):
            cmd['actions'].append({
                'action_type': 'CAPTURE_IMAGE',
                'parameters': {'count': 5, 'interval': 1.0}
            })
            cmd['parameters']['hold_time'] = 5.0  # Hover for 5 seconds
        
        # Calculate timing
        if i > 0:
            distance = euclidean_distance(
                path[i-1]['position'],
                waypoint['position']
            )
            cmd['timing'] = {
                'eta': commands[-1]['timing']['eta'] + distance / cmd['parameters']['velocity'],
                'execution_window': 60.0
            }
        else:
            cmd['timing'] = {'eta': 10.0, 'execution_window': 30.0}
        
        commands.append(cmd)
    
    # Final command: Land or return home
    commands.append({
        'command_id': len(commands),
        'command_type': 'LAND',
        'parameters': {
            'position': path[-1]['position'],
            'velocity': 0.5  # Slow descent
        }
    })
    
    return {
        'drone_id': drone_id,
        'command_sequence': commands,
        'mission_metadata': calculate_metadata(commands)
    }
```

#### 7.4 Configuration

```yaml
command_generation:
  default_velocity: 3.0         # m/s
  default_yaw: 0.0              # degrees (North)
  waypoint_acceptance: 1.0      # meters
  
  takeoff:
    altitude: 10.0              # meters
    velocity: 1.0               # m/s
  
  landing:
    velocity: 0.5               # m/s
    final_altitude: 0.2         # meters
  
  actions:
    image_capture:
      enabled: true
      count: 5
      interval: 1.0             # seconds
    video_recording:
      enabled: false            # Not in V1
  
  mavlink:
    frame: MAV_FRAME_LOCAL_NED  # Coordinate frame
    autocontinue: true
```

#### 7.5 Testing Criteria

- ✓ Generate valid command sequence for 10-waypoint path
- ✓ Convert to MAVLink mission items
- ✓ Calculate accurate timing and battery estimates
- ✓ Include takeoff and landing commands
- ✓ Trigger sensor actions at POIs

---

### 8. DGRC BRIDGE (Drone-Ground-ROS Communication)

**Purpose:** Bidirectional communication bridge between ROS2-based ground station and MAVLink-based drones using MAVROS.

#### 8.1 Inputs

**From Ground Station (ROS2):**
```python
{
    "drone_id": "string",
    "message_type": "command|mission|parameter|request",
    "payload": {
        # Command
        "command_sequence": [...],  # From Command Generation
        
        # Or mission upload
        "mission_items": [...],
        
        # Or parameter set
        "parameter_name": "string",
        "parameter_value": any,
        
        # Or data request
        "request_type": "telemetry|status|diagnostics"
    },
    "timestamp": float
}
```

**From Drone (MAVLink/PX4):**
```python
{
    "drone_id": "string",
    "message_type": "telemetry|status|ack|event",
    "data": {
        # Telemetry
        "position": {"x": float, "y": float, "z": float},
        "velocity": {"vx": float, "vy": float, "vz": float},
        "attitude": {"roll": float, "pitch": float, "yaw": float},
        "battery": {"voltage": float, "current": float, "remaining": float},
        
        # Status
        "mode": "string",
        "armed": bool,
        "system_status": "string",
        
        # Acknowledgment
        "command_ack": {"command_id": int, "result": "success|failed"},
        
        # Events
        "event_type": "waypoint_reached|mission_complete|error"
    },
    "timestamp": float
}
```

#### 8.2 Outputs

**To Drone (MAVLink):**
- Mission waypoints (MISSION_ITEM_INT)
- Commands (COMMAND_LONG, COMMAND_INT)
- Setpoints (SET_POSITION_TARGET_LOCAL_NED)
- Mode changes (SET_MODE)
- Arm/disarm (COMMAND_LONG)

**To Ground Station (ROS2 Topics):**
- Telemetry streams
- Status updates
- Acknowledgments
- Error notifications

#### 8.3 ROS2-MAVLink Message Mapping

```python
# ROS2 → MAVLink
ros_to_mavlink_map = {
    'geometry_msgs/PoseStamped': 'SET_POSITION_TARGET_LOCAL_NED',
    'geometry_msgs/TwistStamped': 'SET_POSITION_TARGET_LOCAL_NED',
    'mavros_msgs/CommandLong': 'COMMAND_LONG',
    'mavros_msgs/Waypoint': 'MISSION_ITEM_INT',
    'std_msgs/String': 'STATUSTEXT'
}

# MAVLink → ROS2
mavlink_to_ros_map = {
    'LOCAL_POSITION_NED': 'geometry_msgs/PoseStamped',
    'GLOBAL_POSITION_INT': 'sensor_msgs/NavSatFix',
    'ATTITUDE': 'geometry_msgs/QuaternionStamped',
    'BATTERY_STATUS': 'sensor_msgs/BatteryState',
    'STATUSTEXT': 'mavros_msgs/StatusText',
    'COMMAND_ACK': 'mavros_msgs/CommandAck'
}
```

#### 8.4 Implementation (MAVROS Integration)

```python
class DGRCBridge:
    def __init__(self, drone_id):
        self.drone_id = drone_id
        self.mavros_namespace = f'/drone{drone_id}/mavros'
        
        # ROS2 Publishers (from drone)
        self.pose_pub = self.create_publisher(
            'PoseStamped', 
            f'/{drone_id}/pose'
        )
        self.battery_pub = self.create_publisher(
            'BatteryState',
            f'/{drone_id}/battery'
        )
        
        # ROS2 Subscribers (to drone)
        self.cmd_sub = self.create_subscription(
            'CommandSequence',
            f'/{drone_id}/commands',
            self.command_callback
        )
        
        # MAVROS Service Clients
        self.mission_push = self.create_client(
            'WaypointPush',
            f'{self.mavros_namespace}/mission/push'
        )
        self.arming = self.create_client(
            'CommandBool',
            f'{self.mavros_namespace}/cmd/arming'
        )
        self.set_mode = self.create_client(
            'SetMode',
            f'{self.mavros_namespace}/set_mode'
        )
    
    def command_callback(self, msg):
        """Handle incoming command sequences"""
        # Convert to MAVROS mission
        mission = self.convert_to_mavros_mission(msg.command_sequence)
        
        # Upload mission
        self.upload_mission(mission)
        
        # Arm and start
        self.arm_drone()
        self.set_auto_mode()
    
    def upload_mission(self, mission):
        """Upload mission waypoints to drone"""
        req = WaypointPush.Request()
        req.waypoints = mission
        
        future = self.mission_push.call_async(req)
        # Handle response...
    
    def telemetry_callback(self, msg):
        """Forward telemetry to ground station"""
        # Transform and republish
        pose_msg = self.mavlink_to_ros_pose(msg)
        self.pose_pub.publish(pose_msg)
```

#### 8.5 Configuration

```yaml
dgrc_bridge:
  mavros_namespace: "/mavros"
  connection:
    protocol: "udp"             # udp|tcp|serial
    port: 14550
    baudrate: 57600             # For serial
    timeout: 5.0                # seconds
  
  telemetry:
    position_rate: 10.0         # Hz
    attitude_rate: 20.0         # Hz
    battery_rate: 1.0           # Hz
    status_rate: 2.0            # Hz
  
  heartbeat:
    enabled: true
    interval: 1.0               # seconds
    timeout: 5.0                # Consider disconnected after
  
  retry:
    max_attempts: 3
    interval: 2.0               # seconds
```

#### 8.6 Error Handling

| Error | Detection | Recovery |
|-------|-----------|----------|
| Connection lost | No heartbeat >5s | Attempt reconnection, trigger RTH |
| Command rejected | COMMAND_ACK(failed) | Retry or alert operator |
| Mission upload failed | Service call failure | Retry, check drone mode |
| Timeout | No response within 5s | Cancel and retry |

#### 8.7 Testing Criteria

- ✓ Establish connection with simulated PX4
- ✓ Upload 10-waypoint mission
- ✓ Receive telemetry at 10Hz
- ✓ Detect connection loss within 5 seconds
- ✓ Successfully arm/disarm drone

---

### 9. TELEMETRY & PX4 MODULE

**Purpose:** Low-level interface with PX4 autopilot for flight control, sensor data acquisition, and state monitoring.

#### 9.1 Inputs

**Flight Commands (from DGRC):**
- Position setpoints (OFFBOARD mode)
- Mission waypoints (AUTO.MISSION mode)
- Manual commands (arm, disarm, takeoff, land, RTH)

**Configuration Parameters:**
```python
{
    "flight_mode": "OFFBOARD|AUTO.MISSION|POSITION|MANUAL",
    "home_position": {"lat": float, "lon": float, "alt": float},
    "geofence": {
        "enabled": bool,
        "boundary": [{"lat": float, "lon": float}],
        "max_altitude": float
    },
    "safety": {
        "battery_failsafe": float,  # % threshold
        "rc_loss_timeout": float,   # seconds
        "datalink_loss_timeout": float
    }
}
```

#### 9.2 Outputs

**High-Frequency Telemetry (10-50 Hz):**
```python
{
    "drone_id": "string",
    "timestamp": float,
    
    "position": {
        "local": {"x": float, "y": float, "z": float},
        "global": {"lat": float, "lon": float, "alt": float},
        "relative_alt": float
    },
    
    "velocity": {
        "linear": {"vx": float, "vy": float, "vz": float},
        "ground_speed": float,
        "air_speed": float
    },
    
    "attitude": {
        "quaternion": {"w": float, "x": float, "y": float, "z": float},
        "euler": {"roll": float, "pitch": float, "yaw": float},
        "angular_velocity": {"wx": float, "wy": float, "wz": float}
    },
    
    "battery": {
        "voltage": float,           # V
        "current": float,           # A
        "remaining": float,         # %
        "time_remaining": float     # minutes
    },
    
    "gps": {
        "fix_type": int,            # 0=no fix, 3=3D fix
        "satellites": int,
        "hdop": float,
        "vdop": float
    },
    
    "system": {
        "mode": "string",
        "armed": bool,
        "system_status": "string",
        "errors": [\"string\"]
    }
}
```

**Low-Frequency Status (1-2 Hz):**
```python
{
    "diagnostics": {
        "cpu_load": float,          # %
        "memory_usage": float,      # %
        "storage_available": float, # MB
        "uptime": float             # seconds
    },
    
    "sensors": {
        "imu": {"status": "ok|degraded|error", "temp": float},
        "gps": {"status": "ok|degraded|error"},
        "compass": {"status": "ok|degraded|error"},
        "barometer": {"status": "ok|degraded|error"}
    }
}
```

**Events:**
```python
{
    "event_type": "waypoint_reached|mission_started|mission_complete|" +
                  "landed|error|warning",
    "description": "string",
    "severity": "info|warning|error|critical",
    "timestamp": float
}
```

#### 9.3 PX4 Integration

**MAVLink Message Handlers:**
```python
class PX4Interface:
    def __init__(self):
        self.mavlink_handlers = {
            'HEARTBEAT': self.handle_heartbeat,
            'LOCAL_POSITION_NED': self.handle_local_position,
            'GLOBAL_POSITION_INT': self.handle_global_position,
            'ATTITUDE': self.handle_attitude,
            'BATTERY_STATUS': self.handle_battery,
            'GPS_RAW_INT': self.handle_gps,
            'MISSION_ITEM_REACHED': self.handle_waypoint_reached,
            'STATUSTEXT': self.handle_statustext
        }
    
    def handle_mavlink_message(self, msg):
        \"\"\"Route incoming MAVLink messages\"\"\"
        msg_type = msg.get_type()
        if msg_type in self.mavlink_handlers:
            self.mavlink_handlers[msg_type](msg)
    
    def send_position_setpoint(self, position, velocity, yaw):
        \"\"\"Send position setpoint in OFFBOARD mode\"\"\"
        msg = self.mavlink_connection.mav.set_position_target_local_ned_encode(
            0,  # time_boot_ms
            0, 1,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,  # type_mask (position + yaw)
            position['x'], position['y'], position['z'],
            velocity['vx'], velocity['vy'], velocity['vz'],
            0, 0, 0,  # acceleration (ignored)
            yaw, 0  # yaw, yaw_rate
        )
        self.mavlink_connection.mav.send(msg)
```

#### 9.4 Flight Modes (PX4)

| Mode | Description | Use Case |
|------|-------------|----------|
| MANUAL | Direct pilot control | Testing, emergency override |
| POSITION | Position hold with GPS | Idle hover |
| AUTO.MISSION | Follow uploaded mission | V1 primary mode |
| OFFBOARD | External control (ROS) | Advanced custom control |
| RETURN | Return to launch | Failsafe, mission end |
| LAND | Autonomous landing | Mission termination |

#### 9.5 Configuration

```yaml
px4_interface:
  connection:
    device: "/dev/ttyACM0"      # Serial device
    baudrate: 921600
    
  telemetry_streams:
    position: {enabled: true, rate: 10}    # Hz
    attitude: {enabled: true, rate: 20}
    battery: {enabled: true, rate: 1}
    gps: {enabled: true, rate: 5}
    
  flight_parameters:
    default_mode: "AUTO.MISSION"
    takeoff_altitude: 10.0      # meters
    land_speed: 0.5             # m/s
    rtl_altitude: 30.0          # meters
    
  safety:
    battery_failsafe: 15.0      # %
    battery_warning: 30.0       # %
    geofence_action: "RTL"      # RTL|LAND|HOLD
    rc_loss_action: "HOLD"
    datalink_loss_action: "RTL"
    datalink_timeout: 10.0      # seconds
```

#### 9.6 Safety Features

**Automatic Failsafes:**
1. **Low Battery:** RTL when < 15%
2. **GPS Loss:** Hold position (if possible), else land
3. **Geofence Breach:** RTL
4. **Communication Loss:** Hold for 10s, then RTL
5. **Critical Error:** Emergency land

#### 9.7 Testing Criteria

- ✓ Establish MAVLink connection with PX4
- ✓ Receive telemetry at configured rates
- ✓ Successfully arm and takeoff
- ✓ Execute 5-waypoint mission
- ✓ Detect and handle low battery condition
- ✓ Successful autonomous landing

---

## System Integration

### 10.1 Communication Architecture

**ROS2 Topic Structure (Optimized for V1):**

**Forward Data Pipeline (Unidirectional):**
```
/scanning/data → /mapping/occupancy_grid → /poi/poi_list → 
/task_allocation/assignments → /path_planning/path/{drone_id} → 
/command_generation/commands/{drone_id} → /dgrc/mavlink_out
```

**Bidirectional Telemetry:**
```
/dgrc/mavlink_out → [PX4]
[PX4] → /dgrc/telemetry/{drone_id}
/dgrc/telemetry/{drone_id} → /drone/{drone_id}/pose,velocity,battery
```

**Status to Orchestrator (All Modules):**
```
/orchestrator/command (pub)           → All modules subscribe
/{module_name}/status (pub)           → Orchestrator subscribes
/{module_name}/progress (pub)         → Orchestrator subscribes (optional)
```

**Topic Details:**
```
Data Flow (→):
  /scanning/data              → SensorData
  /mapping/occupancy_grid     → OccupancyGrid (nav_msgs)
  /mapping/obstacles          → ObstacleArray
  /poi/poi_list              → POIArray
  /task_allocation/assignments → TaskAssignmentArray
  /path_planning/path/{id}    → Path (nav_msgs)
  /command_generation/commands/{id} → CommandSequence
  
Telemetry (↔):
  /dgrc/mavlink_out          → MAVLinkMessage
  /dgrc/telemetry/{id}       → DroneTelemetry
  /drone/{id}/pose           → PoseStamped
  /drone/{id}/velocity       → TwistStamped
  /drone/{id}/battery        → BatteryState
  
Control (↔):
  /orchestrator/command      → OrchestratorCommand (to all)
  /{module}/status          → ModuleStatus (from each)
  
Safety (↔):
  /safety/alerts            → SafetyAlert (broadcast)
  /safety/override          → EmergencyCommand
```

**Service Interfaces:**
```
/scan/generate_plan
/scan/execute
/mapping/build_map
/mapping/get_map
/poi/detect
/task_allocation/allocate
/path_planning/plan_path
/command_generation/generate_commands
```

### 10.2 Data Flow Sequence

**Complete Mission Execution:**
```
1. Orchestrator → Scanning: "Start scan mission"
2. Scanning → Mapping: Raw sensor data stream
3. Mapping → Orchestrator: Occupancy grid ready
4. Orchestrator → POI: "Detect POIs from map"
5. POI → Orchestrator: POI list (N POIs)
6. Orchestrator → Task Allocation: "Assign N POIs to M drones"
7. Task Allocation → Orchestrator: Assignment mapping
8. Orchestrator → Path Planning: N path requests (for-loop)
   - For each drone:
     a. Path Planning: Run A*
     b. Path Planning → Command Generation: Planned path
     c. Command Generation → DGRC: Command sequence
     d. DGRC → PX4: MAVLink mission
9. PX4 → DGRC → Orchestrator: Telemetry + mission progress
10. Orchestrator: Monitor until all missions complete
```

### 10.3 Timing and Synchronization

**Module Processing Times (Expected V1):**
- Orchestrator: < 100ms per state transition
- Scanning: Variable (depends on area size)
- Mapping: 1-5 seconds (for 100x100m area)
- POI Detection: < 1 second
- Task Allocation: < 1 second (10 POIs, 3 drones)
- Path Planning: 2-5 seconds per path
- Command Generation: < 500ms
- DGRC: < 100ms latency
- Total Planning Pipeline: 10-30 seconds (excluding scan time)

**Update Rates:**
- System status: 10 Hz
- Telemetry: 10-20 Hz
- Map updates: 1 Hz (batch mode)
- Path visualization: 2 Hz

### 10.4 State Synchronization

All modules maintain synchronized state through:
- Timestamped messages (Unix time)
- Mission ID tracking
- Sequence numbers for commands
- State machine coordination via Orchestrator

---

## Implementation Guidelines

### 11.1 Technology Stack

**Core Framework:**
- ROS2 Humble (recommended) or Foxy
- Python 3.8+ (primary language for V1)
- C++ (optional, for performance-critical modules)

**Key Libraries:**
```yaml
dependencies:
  # ROS2
  - rclpy
  - std_msgs
  - geometry_msgs
  - sensor_msgs
  - nav_msgs
  - visualization_msgs
  
  # Path Planning
  - numpy
  - scipy
  - opencv-python (for mapping)
  
  # Communication
  - mavros
  - mavros_msgs
  - pymavlink
  
  # Visualization
  - matplotlib
  - rviz2
  
  # Testing
  - pytest
  - unittest
```

**Development Tools:**
- VS Code with ROS extension
- Git for version control
- Docker for deployment (optional)
- QGroundControl for drone monitoring
- Gazebo/PX4-SITL for simulation

### 11.2 Module Development Template

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# Import other message types

class ModuleNameNode(Node):
    def __init__(self):
        super().__init__('module_name_node')
        
        # Parameters
        self.declare_parameter('param_name', default_value)
        self.param_value = self.get_parameter('param_name').value
        
        # Publishers
        self.status_pub = self.create_publisher(
            String, '/module_name/status', 10
        )
        
        # Subscribers
        self.cmd_sub = self.create_subscription(
            String, '/orchestrator/command',
            self.command_callback, 10
        )
        
        # Services
        self.service = self.create_service(
            ServiceType, '/module_name/service',
            self.service_callback
        )
        
        # Timers
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # State
        self.state = 'idle'
        
        self.get_logger().info('Module initialized')
    
    def command_callback(self, msg):
        \"\"\"Handle commands from orchestrator\"\"\"
        pass
    
    def service_callback(self, request, response):
        \"\"\"Handle service requests\"\"\"
        pass
    
    def timer_callback(self):
        \"\"\"Periodic processing\"\"\"
        pass
    
    def publish_status(self):
        \"\"\"Publish module status\"\"\"
        msg = String()
        msg.data = self.state
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ModuleNameNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 11.3 Configuration Management

**Centralized Configuration (YAML):**
```
swarm_config/
├── config/
│   ├── orchestrator.yaml
│   ├── scanning.yaml
│   ├── mapping.yaml
│   ├── poi.yaml
│   ├── task_allocation.yaml
│   ├── path_planning.yaml
│   ├── command_generation.yaml
│   ├── dgrc.yaml
│   └── px4.yaml
├── launch/
│   ├── full_system.launch.py
│   ├── ground_station.launch.py
│   └── simulation.launch.py
└── params/
    └── drones.yaml  # Per-drone configuration
```

**Launch File Template:**
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value='config/default.yaml'),
        
        Node(
            package='swarm_system',
            executable='orchestrator_node',
            name='orchestrator',
            parameters=[LaunchConfiguration('config_file')],
            output='screen'
        ),
        
        Node(
            package='swarm_system',
            executable='scanning_node',
            name='scanning',
            parameters=[LaunchConfiguration('config_file')],
            output='screen'
        ),
        
        # Add other nodes...
    ])
```

### 11.4 Error Handling Strategy

**Hierarchical Error Handling:**
1. **Module Level:** Try to recover locally (retry, use defaults)
2. **Orchestrator Level:** Coordinate recovery across modules
3. **System Level:** Graceful degradation or safe abort

**Error Logging:**
```python
self.get_logger().debug('Detailed debug information')
self.get_logger().info('Normal operation event')
self.get_logger().warn('Recoverable issue detected')
self.get_logger().error('Error requiring attention')
self.get_logger().fatal('Critical failure, system shutdown')
```

### 11.5 Version Control Structure

```
swarm-drone-system/
├── src/
│   ├── orchestrator/
│   ├── scanning/
│   ├── mapping/
│   ├── poi/
│   ├── task_allocation/
│   ├── path_planning/
│   ├── command_generation/
│   ├── dgrc/
│   └── common/  # Shared utilities
├── config/
├── launch/
├── test/
├── docs/
├── simulation/
├── README.md
├── package.xml
└── setup.py
```

---

## Testing and Validation

### 12.1 Unit Testing

**Per-Module Tests:**
```python
import pytest
from swarm_system.path_planning import AStarPlanner

def test_astar_simple_path():
    \"\"\"Test A* finds path in simple grid\"\"\"
    grid = create_empty_grid(10, 10)
    planner = AStarPlanner(grid)
    
    start = (0, 0)
    goal = (9, 9)
    
    path = planner.plan(start, goal)
    
    assert path is not None
    assert len(path) > 0
    assert path[0] == start
    assert path[-1] == goal

def test_astar_with_obstacle():
    \"\"\"Test A* avoids obstacles\"\"\"
    grid = create_empty_grid(10, 10)
    grid[5, :] = 100  # Horizontal wall
    
    planner = AStarPlanner(grid)
    path = planner.plan((0, 5), (9, 5))
    
    # Path should go around the wall
    assert all(grid[int(p[1]), int(p[0])] < 50 for p in path)
```

### 12.2 Integration Testing

**Module Chain Tests:**
```python
def test_mapping_to_poi_pipeline():
    \"\"\"Test Mapping → POI detection pipeline\"\"\"
    # 1. Generate mock sensor data
    sensor_data = generate_mock_lidar_data()
    
    # 2. Build map
    mapper = MappingModule()
    occupancy_grid = mapper.build_map(sensor_data)
    
    # 3. Detect POIs
    poi_detector = POIModule()
    pois = poi_detector.detect(occupancy_grid)
    
    # 4. Validate
    assert len(pois) > 0
    assert all('position' in poi for poi in pois)
```

### 12.3 System-Level Testing (SITL)

**Simulation-in-the-Loop Tests:**
```bash
# 1. Launch PX4 SITL
cd PX4-Autopilot
make px4_sitl gazebo

# 2. Launch ROS2 system
ros2 launch swarm_system full_system.launch.py simulation:=true

# 3. Execute test mission
ros2 run swarm_system test_mission_executor --mission simple_scan

# 4. Validate results
ros2 run swarm_system validate_mission_results
```

**Test Scenarios:**
1. **Basic Flight:** Single drone, 4-waypoint mission
2. **Area Scan:** 50x50m grid scan with mapping
3. **POI Inspection:** Detect and visit 5 POIs
4. **Multi-Drone:** 3 drones, coordinate task allocation
5. **Failsafe:** Low battery trigger RTL
6. **Recovery:** Handle path planning failure

### 12.4 Hardware-in-the-Loop (HIL)

**Progression to Real Hardware:**
1. **Stage 1:** Full simulation (Gazebo + PX4 SITL)
2. **Stage 2:** HIL (real autopilot, simulated environment)
3. **Stage 3:** Bench testing (tethered drone, no flight)
4. **Stage 4:** Indoor flight tests (controlled environment)
5. **Stage 5:** Outdoor flight tests (real mission scenarios)

### 12.5 Performance Benchmarks

**Acceptance Criteria:**
| Metric | Target | Measured |
|--------|--------|----------|
| Path planning time | < 5s | __ |
| Mapping latency | < 2s | __ |
| Task allocation | < 1s | __ |
| Telemetry rate | 10 Hz | __ |
| Command latency | < 200ms | __ |
| System memory | < 2GB | __ |
| CPU usage | < 50% | __ |

---

## Deployment

### 13.1 Hardware Requirements

**Ground Station:**
- CPU: Intel i5 or equivalent (4 cores minimum)
- RAM: 8GB minimum, 16GB recommended
- Storage: 50GB SSD
- Network: WiFi 5GHz or Ethernet
- OS: Ubuntu 20.04/22.04 LTS

**Drone (Companion Computer):**
- Raspberry Pi 4 (4GB RAM) or equivalent
- MicroSD card: 32GB minimum
- PX4-compatible flight controller (Pixhawk 4, etc.)
- Telemetry radio: 915MHz or WiFi
- Power: 5V 3A stable supply

### 13.2 Software Installation

**Ground Station Setup:**
```bash
# 1. Install ROS2 Humble
sudo apt install ros-humble-desktop

# 2. Install dependencies
sudo apt install python3-pip python3-colcon-common-extensions
pip3 install numpy scipy opencv-python matplotlib

# 3. Install MAVROS
sudo apt install ros-humble-mavros ros-humble-mavros-extras
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

# 4. Clone and build workspace
mkdir -p ~/swarm_ws/src
cd ~/swarm_ws/src
git clone https://github.com/your-org/swarm-drone-system.git
cd ~/swarm_ws
colcon build
source install/setup.bash

# 5. Configure
cp src/swarm-drone-system/config/default.yaml ~/swarm_config.yaml
# Edit config file as needed
```

**Drone Setup (Companion Computer):**
```bash
# 1. Flash Ubuntu Server 20.04 on Raspberry Pi
# 2. Install ROS2 Base
sudo apt install ros-humble-ros-base

# 3. Install MAVROS
sudo apt install ros-humble-mavros

# 4. Configure serial connection to Pixhawk
# Edit /boot/config.txt, disable Bluetooth UART

# 5. Set up autostart
sudo systemctl enable swarm-drone-agent.service
```

### 13.3 Network Configuration

**Architecture:**
```
Ground Station (192.168.1.100)
    ↕ WiFi/Radio
Drone 1 (192.168.1.101)
    ↕ Serial/USB
PX4 Autopilot (via MAVROS)
```

**Ports:**
- ROS2 DDS: Default (multicast)
- MAVROS: Serial (/dev/ttyACM0, 921600 baud)
- QGC: UDP 14550
- SSH: 22 (for debugging)

### 13.4 Pre-Flight Checklist

**Software:**
- [ ] ROS2 nodes all running (`ros2 node list`)
- [ ] MAVROS connected (`ros2 topic echo /mavros/state`)
- [ ] Telemetry receiving (`ros2 topic hz /drone/D1/pose`)
- [ ] Config file loaded correctly
- [ ] Mission plan validated

**Hardware:**
- [ ] Battery charged (> 90%)
- [ ] GPS lock (> 10 satellites)
- [ ] Propellers secured
- [ ] Radio link good (> -70 dBm)
- [ ] Camera/sensors functional
- [ ] Geofence configured
- [ ] Return home point set

---

## Version 1 Development Principles (Reiterated)

### What Version 1 IS:
✅ **Functional end-to-end system** - All modules work together  
✅ **Surface-level implementation** - Clear inputs/outputs, basic internals  
✅ **Basic algorithms** - A* for planning, greedy for allocation  
✅ **Simple, working solution** - Demonstrable and testable  
✅ **Foundation for future** - Clean architecture for improvements  
✅ **Well-documented** - This comprehensive specification

### What Version 1 IS NOT:
❌ **Optimized or scalable** - No performance tuning yet  
❌ **Handling complex edge cases** - Focused on happy path  
❌ **Multi-drone collision avoidance** - Future version  
❌ **Advanced algorithms or AI** - Keep it simple  
❌ **Production-ready code** - Needs hardening and testing  
❌ **Fault-tolerant** - Basic error handling only

### Development Approach (From Meeting):
1. **Focus on ONE module at a time** - Don't spread too thin
2. **Define clear inputs/outputs first** - Contract-driven development
3. **Make it work, then make it better** - Functional before optimal
4. **Publish progress per module** - Don't wait for everything
5. **Iterate and improve** - V1 is just the beginning

---

## Next Steps and Roadmap

### Immediate (Week 1-2):
1. Set up development environment (ROS2, MAVROS, simulation)
2. Implement Orchestrator skeleton
3. Create mock data generators for testing
4. Implement Path Planning module (A* algorithm)
5. Test in simulation (PX4 SITL + Gazebo)

### Short-term (Week 3-4):
1. Implement Scanning module (grid pattern generation)
2. Implement Mapping module (occupancy grid)
3. Integrate Path Planning → Command Generation → DGRC
4. End-to-end single-drone mission test

### Medium-term (Month 2):
1. Implement POI detection
2. Implement Task Allocation
3. Multi-drone simulation tests
4. Hardware-in-the-loop testing

### Long-term (Month 3+):
1. Field testing with real drones
2. Performance optimization
3. Begin Version 2 planning:
   - Inter-drone collision avoidance
   - Dynamic replanning
   - Advanced allocation algorithms
   - Fault tolerance improvements

---

## Documentation and Knowledge Transfer

### Daily Standup Format (per meeting):
**Each team member reports:**
1. What I completed yesterday
2. What I plan to complete today (quantified targets)
3. Blockers or dependencies

**Minutes of Meeting (MoM):**
- Document decisions made
- Action items with owners
- Technical discussions summary
- Next meeting agenda

### Progress Tracking:
- One markdown file per module (inputs, outputs, status)
- Publish updates incrementally (don't wait)
- Use Git commits with descriptive messages
- Weekly demo videos of progress

---

## Version History

- **Version 1 (Current):** Initial architecture based on meeting discussion (Dec 25, 2025)
- **Future versions:** Will include optimizations, advanced algorithms, and scalability improvements

---

## References

- Meeting discussion: Dec 25, 2025
- Application: Area scanning with drone swarm
- Simulation: Cloudy environment, RViz visualization
- Hardware: PX4 autopilot, QGroundControl
