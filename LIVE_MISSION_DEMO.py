#!/usr/bin/env python3
"""
Live Mission Demonstration - Orchestrator V1
Shows realistic mission operation with detailed progress tracking
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum, auto
import time
import json
import os
import random
from datetime import datetime

class MissionState(Enum):
    """Mission states"""
    INIT = auto()
    READY = auto()
    SCANNING = auto()
    MAPPING = auto()
    POI_DETECTION = auto()
    TASK_ALLOCATION = auto()
    PATH_PLANNING = auto()
    COMMAND_GENERATION = auto()
    DRONE_EXECUTION = auto()
    COMPLETED = auto()
    PAUSED = auto()
    ERROR = auto()
    ABORTED = auto()


class LiveMissionDemo(Node):
    """
    Live Mission Demonstration with realistic data and progress tracking
    """
    
    def __init__(self):
        super().__init__('live_mission_demo')
        
        # Mission configuration
        self.scan_area = {
            'x_min': 0, 'y_min': 0,
            'x_max': 100, 'y_max': 100,
            'altitude': 20
        }
        self.num_drones = 3
        self.drone_ids = [f'drone_{i+1}' for i in range(self.num_drones)]
        
        # Current state
        self.state = MissionState.INIT
        self.previous_state = None
        
        # Mission data
        self.mission_id = ''
        self.mission_start_time = None
        self.scan_data = []
        self.occupancy_grid = None
        self.poi_list = []
        self.task_assignments = {}
        self.drone_paths = {}
        self.drone_commands = {}
        
        # Progress tracking
        self.scan_progress = 0.0
        self.mapping_progress = 0.0
        self.execution_progress = {}
        
        # Drone telemetry
        self.drone_telemetry = {
            drone_id: {
                'position': {'x': 0, 'y': 0, 'z': 0},
                'battery': 100.0,
                'status': 'IDLE',
                'waypoint_index': 0
            } for drone_id in self.drone_ids
        }
        
        # Error handling
        self.error_count = 0
        self.max_retries = 3
        self.error_message = ''
        
        # Control flags
        self.pause_requested = False
        self.abort_requested = False
        self.ready_start_time = None
        
        # Visualization
        self.viz_file = os.path.join(os.path.dirname(__file__), 'orchestrator_state.json')
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/orchestrator/status', 10)
        
        # Command subscriber
        self.cmd_sub = self.create_subscription(
            String, '/orchestrator/command', self.command_callback, 10
        )
        
        self.get_logger().info('='*60)
        self.get_logger().info('  LIVE MISSION DEMONSTRATION - ORCHESTRATOR V1')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Number of drones: {self.num_drones}')
        self.get_logger().info(f'Scan area: {self.scan_area["x_max"]}x{self.scan_area["y_max"]}m')
        self.get_logger().info('='*60)
    
    def command_callback(self, msg):
        """Handle external commands"""
        cmd = msg.data.lower()
        if cmd == 'pause':
            self.pause_requested = True
        elif cmd == 'resume':
            if self.state == MissionState.PAUSED:
                self.state = self.previous_state
                self.pause_requested = False
        elif cmd == 'abort':
            self.abort_requested = True
    
    def run(self):
        """Main state machine loop"""
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            
            # Check for pause/abort
            if self.pause_requested and self.state not in [MissionState.PAUSED, MissionState.COMPLETED]:
                self.previous_state = self.state
                self.state = MissionState.PAUSED
            
            if self.abort_requested:
                self.state = MissionState.ABORTED
            
            # Execute current state
            if self.state == MissionState.INIT:
                self.state_init()
            elif self.state == MissionState.READY:
                self.state_ready()
            elif self.state == MissionState.SCANNING:
                self.state_scanning()
            elif self.state == MissionState.MAPPING:
                self.state_mapping()
            elif self.state == MissionState.POI_DETECTION:
                self.state_poi_detection()
            elif self.state == MissionState.TASK_ALLOCATION:
                self.state_task_allocation()
            elif self.state == MissionState.PATH_PLANNING:
                self.state_path_planning()
            elif self.state == MissionState.COMMAND_GENERATION:
                self.state_command_generation()
            elif self.state == MissionState.DRONE_EXECUTION:
                self.state_drone_execution()
            elif self.state == MissionState.PAUSED:
                self.state_paused()
            elif self.state == MissionState.ERROR:
                self.state_error()
            elif self.state == MissionState.COMPLETED:
                self.state_completed()
                break
            elif self.state == MissionState.ABORTED:
                self.state_aborted()
                break
            
            # Publish status
            self.publish_status()
            
            time.sleep(0.1)
    
    # ============ STATE IMPLEMENTATIONS ============
    
    def state_init(self):
        """Initialize system"""
        self.get_logger().info('⚙️  INITIALIZING SYSTEM...')
        
        modules = ['scanning', 'mapping', 'poi', 'task_allocation',
                   'path_planning', 'command_gen', 'dgrc', 'telemetry']
        
        for i, module in enumerate(modules):
            time.sleep(0.3)
            self.get_logger().info(f'   [{i+1}/{len(modules)}] Checking {module}... ✓')
        
        self.mission_id = f"MISSION_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.mission_start_time = time.time()
        
        self.get_logger().info(f'✓ System initialized: {self.mission_id}')
        self.get_logger().info('')
        self.state = MissionState.READY
    
    def state_ready(self):
        """Wait for start"""
        if self.ready_start_time is None:
            self.ready_start_time = time.time()
            self.get_logger().info('🔵 READY - Mission starting in 3 seconds...')
            for i in range(3, 0, -1):
                time.sleep(1)
                self.get_logger().info(f'   {i}...')
        
        self.get_logger().info('🚁 MISSION START!')
        self.get_logger().info('')
        self.ready_start_time = None
        self.state = MissionState.SCANNING
    
    def state_scanning(self):
        """Execute area scanning with progress"""
        if self.scan_progress == 0:
            self.get_logger().info('📡 SCANNING PHASE')
            self.get_logger().info(f'   Area: {self.scan_area["x_max"]}x{self.scan_area["y_max"]}m @ {self.scan_area["altitude"]}m altitude')
            self.get_logger().info(f'   Drones: {", ".join(self.drone_ids)}')
            self.get_logger().info('')
        
        # Simulate scanning with progress
        while self.scan_progress < 100:
            self.scan_progress += 5
            
            # Simulate sensor data collection
            num_readings = random.randint(50, 150)
            self.scan_data.extend([
                {'x': random.uniform(0, 100), 'y': random.uniform(0, 100), 
                 'obstacle': random.random() > 0.8}
                for _ in range(num_readings)
            ])
            
            # Update drone positions
            for drone_id in self.drone_ids:
                self.drone_telemetry[drone_id]['position']['x'] = random.uniform(0, 100)
                self.drone_telemetry[drone_id]['position']['y'] = random.uniform(0, 100)
                self.drone_telemetry[drone_id]['battery'] -= 0.5
                self.drone_telemetry[drone_id]['status'] = 'SCANNING'
            
            self.get_logger().info(f'   Progress: {self.scan_progress:.0f}% | Data points: {len(self.scan_data)}')
            time.sleep(0.3)
        
        self.get_logger().info(f'✓ Scanning complete: {len(self.scan_data)} sensor readings collected')
        self.get_logger().info('')
        self.scan_progress = 0
        self.state = MissionState.MAPPING
    
    def state_mapping(self):
        """Build map with progress"""
        if self.mapping_progress == 0:
            self.get_logger().info('🗺️  MAPPING PHASE')
            self.get_logger().info(f'   Processing {len(self.scan_data)} sensor readings...')
            self.get_logger().info('')
        
        while self.mapping_progress < 100:
            self.mapping_progress += 10
            self.get_logger().info(f'   Building occupancy grid: {self.mapping_progress:.0f}%')
            time.sleep(0.3)
        
        # Create simulated map with obstacles
        num_obstacles = random.randint(5, 15)
        self.occupancy_grid = {
            'resolution': 0.5,
            'size': [200, 200],
            'obstacles': [
                {'id': i, 'x': random.uniform(10, 90), 'y': random.uniform(10, 90),
                 'size': random.uniform(1, 5)}
                for i in range(num_obstacles)
            ]
        }
        
        self.get_logger().info(f'✓ Map built: {num_obstacles} obstacles detected')
        self.get_logger().info('')
        self.mapping_progress = 0
        self.state = MissionState.POI_DETECTION
    
    def state_poi_detection(self):
        """Detect POIs"""
        self.get_logger().info('🎯 POI DETECTION PHASE')
        self.get_logger().info('   Analyzing map for points of interest...')
        time.sleep(0.5)
        
        # Generate POIs based on obstacles
        num_pois = random.randint(4, 8)
        self.poi_list = [
            {
                'id': f'POI_{i+1}',
                'x': random.uniform(10, 90),
                'y': random.uniform(10, 90),
                'priority': random.choice(['HIGH', 'MEDIUM', 'LOW']),
                'type': random.choice(['anomaly', 'target', 'inspection'])
            }
            for i in range(num_pois)
        ]
        
        for poi in self.poi_list:
            self.get_logger().info(f'   ✓ {poi["id"]}: ({poi["x"]:.1f}, {poi["y"]:.1f}) - {poi["priority"]} priority - {poi["type"]}')
        
        self.get_logger().info(f'✓ Found {len(self.poi_list)} POIs')
        self.get_logger().info('')
        self.state = MissionState.TASK_ALLOCATION
    
    def state_task_allocation(self):
        """Allocate POIs to drones"""
        self.get_logger().info('📋 TASK ALLOCATION PHASE')
        self.get_logger().info(f'   Assigning {len(self.poi_list)} POIs to {self.num_drones} drones...')
        time.sleep(0.5)
        
        # Simple round-robin allocation
        for i, poi in enumerate(self.poi_list):
            drone_id = self.drone_ids[i % self.num_drones]
            if drone_id not in self.task_assignments:
                self.task_assignments[drone_id] = []
            self.task_assignments[drone_id].append(poi)
        
        for drone_id, pois in self.task_assignments.items():
            poi_ids = [p['id'] for p in pois]
            self.get_logger().info(f'   ✓ {drone_id}: {len(pois)} POIs → {", ".join(poi_ids)}')
        
        self.get_logger().info(f'✓ Task allocation complete')
        self.get_logger().info('')
        self.state = MissionState.PATH_PLANNING
    
    def state_path_planning(self):
        """Generate paths for all drones"""
        self.get_logger().info('🛤️  PATH PLANNING PHASE (A* Algorithm)')
        
        for drone_id, pois in self.task_assignments.items():
            self.get_logger().info(f'   Planning path for {drone_id}...')
            time.sleep(0.4)
            
            # Generate waypoints
            waypoints = [{'x': 0, 'y': 0, 'z': 20}]  # Start at home
            for poi in pois:
                waypoints.append({'x': poi['x'], 'y': poi['y'], 'z': 20})
            waypoints.append({'x': 0, 'y': 0, 'z': 20})  # Return home
            
            path_length = sum([
                ((waypoints[i+1]['x'] - waypoints[i]['x'])**2 + 
                 (waypoints[i+1]['y'] - waypoints[i]['y'])**2)**0.5
                for i in range(len(waypoints)-1)
            ])
            
            self.drone_paths[drone_id] = {
                'waypoints': waypoints,
                'length': path_length,
                'estimated_time': path_length / 5.0  # 5 m/s average speed
            }
            
            self.get_logger().info(f'      Path: {len(waypoints)} waypoints, {path_length:.1f}m, ETA: {self.drone_paths[drone_id]["estimated_time"]:.1f}s')
        
        self.get_logger().info(f'✓ All paths planned')
        self.get_logger().info('')
        self.state = MissionState.COMMAND_GENERATION
    
    def state_command_generation(self):
        """Generate MAVLink commands"""
        self.get_logger().info('📝 COMMAND GENERATION PHASE')
        
        for drone_id, path in self.drone_paths.items():
            self.get_logger().info(f'   Generating MAVLink commands for {drone_id}...')
            time.sleep(0.3)
            
            commands = []
            commands.append({'cmd': 'MAV_CMD_NAV_TAKEOFF', 'alt': 20})
            for wp in path['waypoints'][1:-1]:
                commands.append({
                    'cmd': 'MAV_CMD_NAV_WAYPOINT',
                    'x': wp['x'], 'y': wp['y'], 'z': wp['z']
                })
            commands.append({'cmd': 'MAV_CMD_NAV_LAND', 'x': 0, 'y': 0})
            
            self.drone_commands[drone_id] = commands
            self.get_logger().info(f'      Generated {len(commands)} commands')
        
        self.get_logger().info(f'✓ All commands generated')
        self.get_logger().info('')
        self.state = MissionState.DRONE_EXECUTION
    
    def state_drone_execution(self):
        """Execute mission with live telemetry"""
        if not self.execution_progress:
            self.get_logger().info('🚁 DRONE EXECUTION PHASE')
            self.get_logger().info('   Mission in progress - monitoring telemetry...')
            self.get_logger().info('')
            
            for drone_id in self.drone_ids:
                self.execution_progress[drone_id] = 0.0
        
        # Simulate mission execution
        all_complete = True
        
        for drone_id in self.drone_ids:
            if self.execution_progress[drone_id] < 100:
                all_complete = False
                self.execution_progress[drone_id] += random.uniform(3, 8)
                
                if self.execution_progress[drone_id] > 100:
                    self.execution_progress[drone_id] = 100
                
                # Update telemetry
                path = self.drone_paths[drone_id]
                progress_idx = int((self.execution_progress[drone_id] / 100) * len(path['waypoints']))
                if progress_idx < len(path['waypoints']):
                    wp = path['waypoints'][progress_idx]
                    self.drone_telemetry[drone_id]['position'] = wp
                    self.drone_telemetry[drone_id]['waypoint_index'] = progress_idx
                
                self.drone_telemetry[drone_id]['battery'] -= 0.3
                self.drone_telemetry[drone_id]['status'] = 'EXECUTING'
        
        # Display telemetry
        self.get_logger().info('   Drone Telemetry:')
        for drone_id in self.drone_ids:
            pos = self.drone_telemetry[drone_id]['position']
            battery = self.drone_telemetry[drone_id]['battery']
            progress = self.execution_progress[drone_id]
            wp_idx = self.drone_telemetry[drone_id]['waypoint_index']
            total_wps = len(self.drone_paths[drone_id]['waypoints'])
            
            self.get_logger().info(
                f'   {drone_id}: Progress {progress:.0f}% | '
                f'WP {wp_idx}/{total_wps} | '
                f'Pos ({pos["x"]:.1f}, {pos["y"]:.1f}, {pos["z"]:.1f}) | '
                f'Battery {battery:.1f}%'
            )
        
        self.get_logger().info('')
        time.sleep(0.5)
        
        if all_complete:
            self.get_logger().info('✓ All drones completed their missions')
            self.get_logger().info('')
            self.state = MissionState.COMPLETED
    
    def state_paused(self):
        """Paused state"""
        time.sleep(0.1)
    
    def state_error(self):
        """Error handling"""
        self.get_logger().error(f'❌ ERROR: {self.error_message}')
        self.error_count += 1
        
        if self.error_count < self.max_retries:
            self.get_logger().info(f'   Retry {self.error_count}/{self.max_retries}')
            self.state = MissionState.INIT
        else:
            self.state = MissionState.ABORTED
    
    def state_completed(self):
        """Mission completed"""
        elapsed = time.time() - self.mission_start_time
        
        self.get_logger().info('='*60)
        self.get_logger().info('  ✓ MISSION COMPLETED SUCCESSFULLY!')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Mission ID: {self.mission_id}')
        self.get_logger().info(f'Duration: {elapsed:.1f} seconds')
        self.get_logger().info(f'POIs visited: {len(self.poi_list)}')
        self.get_logger().info(f'Drones used: {self.num_drones}')
        
        for drone_id in self.drone_ids:
            battery = self.drone_telemetry[drone_id]['battery']
            self.get_logger().info(f'   {drone_id}: Battery remaining {battery:.1f}%')
        
        self.get_logger().info('='*60)
    
    def state_aborted(self):
        """Mission aborted"""
        self.get_logger().error('='*60)
        self.get_logger().error('  ❌ MISSION ABORTED')
        self.get_logger().error('='*60)
        self.get_logger().error(f'Reason: {self.error_message}')
        self.get_logger().error('='*60)
    
    def publish_status(self):
        """Publish status"""
        status_msg = String()
        status_msg.data = f'{self.state.name}|{self.mission_id}|errors:{self.error_count}'
        self.status_pub.publish(status_msg)
        
        # Save for visualization
        state_data = {
            'state': self.state.name,
            'mission_id': self.mission_id,
            'error_count': self.error_count,
            'timestamp': time.time(),
            'scan_progress': self.scan_progress,
            'mapping_progress': self.mapping_progress,
            'execution_progress': self.execution_progress,
            'telemetry': self.drone_telemetry
        }
        
        try:
            with open(self.viz_file, 'w') as f:
                json.dump(state_data, f)
        except Exception as e:
            pass


def main(args=None):
    rclpy.init(args=args)
    
    demo = LiveMissionDemo()
    
    try:
        demo.run()
    except KeyboardInterrupt:
        pass
    finally:
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
