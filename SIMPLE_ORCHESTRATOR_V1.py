#!/usr/bin/env python3
"""
Simple Orchestrator State Machine - Version 1
No SMACH dependency - just pure Python class

Philosophy: Make it work first, optimize later
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum, auto
import time
import json
import os

class MissionState(Enum):
    """Mission states - simple enum"""
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


class SimpleOrchestrator(Node):
    """
    Simple state machine orchestrator without SMACH
    Easy to understand, debug, and modify
    """
    
    def __init__(self):
        super().__init__('simple_orchestrator')
        
        # Current state
        self.state = MissionState.INIT
        self.previous_state = None
        
        # Mission data (shared across states)
        self.mission_id = ''
        self.scan_area = {}
        self.scan_data = None
        self.occupancy_grid = None
        self.poi_list = []
        self.task_assignments = {}
        self.drone_paths = {}
        self.drone_commands = {}
        
        # Error handling
        self.error_count = 0
        self.max_retries = 3
        self.error_message = ''
        
        # Control flags
        self.pause_requested = False
        self.abort_requested = False
        
        # Timing
        self.ready_start_time = None
        self.auto_start_delay = 2.0  # seconds
        
        # Visualization output file
        self.viz_file = os.path.join(os.path.dirname(__file__), 'orchestrator_state.json')
        
        # Create service clients (add your actual services)
        # self.scan_client = self.create_client(...)
        # self.mapping_client = self.create_client(...)
        # etc.
        
        # Status publisher
        self.status_pub = self.create_publisher(String, '/orchestrator/status', 10)
        
        # Command subscriber
        self.cmd_sub = self.create_subscription(
            String,
            '/orchestrator/command',
            self.command_callback,
            10
        )
        
        self.get_logger().info('Simple Orchestrator initialized')
    
    def command_callback(self, msg):
        """Handle external commands"""
        cmd = msg.data.lower()
        if cmd == 'start':
            if self.state == MissionState.READY:
                self.get_logger().info('Start command received')
        elif cmd == 'pause':
            self.pause_requested = True
            self.get_logger().info('Pause requested')
        elif cmd == 'resume':
            if self.state == MissionState.PAUSED:
                self.state = self.previous_state
                self.pause_requested = False
                self.get_logger().info('Resuming mission')
        elif cmd == 'abort':
            self.abort_requested = True
            self.get_logger().info('Abort requested')
    
    def run(self):
        """Main state machine loop"""
        
        while rclpy.ok():
            # Spin once to process callbacks
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
            
            # Sleep briefly (100ms loop rate)
            time.sleep(0.1)
    
    # ============ STATE IMPLEMENTATIONS ============
    
    def state_init(self):
        """Initialize system and check all modules"""
        self.get_logger().info('STATE: INIT - Checking modules...')
        
        # Check module availability
        modules = ['scanning', 'mapping', 'poi', 'task_allocation',
                   'path_planning', 'command_gen', 'dgrc', 'telemetry']
        
        all_ready = True
        for module in modules:
            # Check if service exists
            # ready = self.check_module_ready(module)
            # if not ready:
            #     all_ready = False
            #     break
            pass  # V1: Skip detailed checks, assume ready
        
        if all_ready:
            self.mission_id = f"mission_{int(time.time())}"
            self.get_logger().info(f'System initialized: {self.mission_id}')
            self.state = MissionState.READY
        else:
            self.error_message = 'Module initialization failed'
            self.state = MissionState.ERROR
    
    def state_ready(self):
        """Wait for start command"""
        # Log only once when entering this state
        if self.ready_start_time is None:
            self.ready_start_time = time.time()
            self.get_logger().info('STATE: READY - Auto-starting in 2 seconds...')
        
        # Check if enough time has passed to auto-start
        if time.time() - self.ready_start_time >= self.auto_start_delay:
            self.get_logger().info('Auto-start triggered')
            self.ready_start_time = None  # Reset for next time
            self.state = MissionState.SCANNING
    
    def state_scanning(self):
        """Execute scanning phase"""
        self.get_logger().info('STATE: SCANNING - Starting area scan')
        
        try:
            # Call scanning service
            # response = self.scan_client.call(request)
            # self.scan_data = response.data
            
            # V1 Placeholder: simulate scan
            time.sleep(1)
            self.scan_data = {'simulated': True}
            
            self.get_logger().info('Scanning complete')
            self.state = MissionState.MAPPING
            
        except Exception as e:
            self.error_message = f'Scanning failed: {e}'
            self.state = MissionState.ERROR
    
    def state_mapping(self):
        """Build map from scan data"""
        self.get_logger().info('STATE: MAPPING - Building occupancy grid')
        
        try:
            # Call mapping service
            # response = self.mapping_client.call(self.scan_data)
            # self.occupancy_grid = response.grid
            
            # V1 Placeholder
            time.sleep(1)
            self.occupancy_grid = {'simulated': True}
            
            self.get_logger().info('Mapping complete')
            self.state = MissionState.POI_DETECTION
            
        except Exception as e:
            self.error_message = f'Mapping failed: {e}'
            self.state = MissionState.ERROR
    
    def state_poi_detection(self):
        """Detect points of interest"""
        self.get_logger().info('STATE: POI_DETECTION - Identifying POIs')
        
        try:
            # Call POI detection service
            # response = self.poi_client.call(self.occupancy_grid)
            # self.poi_list = response.pois
            
            # V1 Placeholder
            time.sleep(0.5)
            self.poi_list = [{'id': 1, 'x': 10, 'y': 20}, {'id': 2, 'x': 30, 'y': 40}]
            
            if len(self.poi_list) == 0:
                self.get_logger().info('No POIs detected - mission complete')
                self.state = MissionState.COMPLETED
            else:
                self.get_logger().info(f'Found {len(self.poi_list)} POIs')
                self.state = MissionState.TASK_ALLOCATION
            
        except Exception as e:
            self.error_message = f'POI detection failed: {e}'
            self.state = MissionState.ERROR
    
    def state_task_allocation(self):
        """Allocate POIs to drones"""
        self.get_logger().info('STATE: TASK_ALLOCATION - Assigning tasks')
        
        try:
            # Call task allocation service
            # response = self.task_alloc_client.call(self.poi_list)
            # self.task_assignments = response.assignments
            
            # V1 Placeholder: simple assignment
            time.sleep(0.5)
            self.task_assignments = {
                'drone_1': [self.poi_list[0]],
                'drone_2': [self.poi_list[1]] if len(self.poi_list) > 1 else []
            }
            
            self.get_logger().info(f'Tasks allocated to {len(self.task_assignments)} drones')
            self.state = MissionState.PATH_PLANNING
            
        except Exception as e:
            self.error_message = f'Task allocation failed: {e}'
            self.state = MissionState.ERROR
    
    def state_path_planning(self):
        """Generate paths for all drones"""
        self.get_logger().info('STATE: PATH_PLANNING - Planning paths')
        
        try:
            # For each drone, plan path (for-loop, V1 approach)
            for drone_id, tasks in self.task_assignments.items():
                # Call path planning service
                # response = self.path_plan_client.call(drone_id, tasks, self.occupancy_grid)
                # self.drone_paths[drone_id] = response.path
                
                # V1 Placeholder
                self.drone_paths[drone_id] = {'waypoints': [(0,0), (10,10), (20,20)]}
                self.get_logger().info(f'Path planned for {drone_id}')
            
            self.get_logger().info('All paths planned')
            self.state = MissionState.COMMAND_GENERATION
            
        except Exception as e:
            self.error_message = f'Path planning failed: {e}'
            self.state = MissionState.ERROR
    
    def state_command_generation(self):
        """Generate MAVLink commands"""
        self.get_logger().info('STATE: COMMAND_GENERATION - Generating commands')
        
        try:
            # Generate commands for each drone
            for drone_id, path in self.drone_paths.items():
                # response = self.cmd_gen_client.call(drone_id, path)
                # self.drone_commands[drone_id] = response.commands
                
                # V1 Placeholder
                self.drone_commands[drone_id] = {'mavlink_items': [1, 2, 3]}
                self.get_logger().info(f'Commands generated for {drone_id}')
            
            self.get_logger().info('All commands ready')
            self.state = MissionState.DRONE_EXECUTION
            
        except Exception as e:
            self.error_message = f'Command generation failed: {e}'
            self.state = MissionState.ERROR
    
    def state_drone_execution(self):
        """Execute mission on drones"""
        self.get_logger().info('STATE: DRONE_EXECUTION - Executing mission')
        
        try:
            # Send commands to drones via DGRC
            # Monitor telemetry and safety
            # Wait for completion
            
            # V1 Placeholder: simulate execution
            time.sleep(3)
            
            self.get_logger().info('Mission execution complete')
            self.state = MissionState.COMPLETED
            
        except Exception as e:
            self.error_message = f'Drone execution failed: {e}'
            self.state = MissionState.ERROR
    
    def state_paused(self):
        """Mission paused - waiting for resume"""
        # Just wait, transition happens in command_callback
        time.sleep(0.1)    
    
    def state_error(self):
        """Handle errors with retry logic"""
        self.get_logger().error(f'STATE: ERROR - {self.error_message}')
        
        self.error_count += 1
        
        if self.error_count < self.max_retries:
            self.get_logger().info(f'Retry {self.error_count}/{self.max_retries}')
            # Go back to previous state or INIT
            self.state = MissionState.INIT
        else:
            self.get_logger().error('Max retries reached - aborting')
            self.state = MissionState.ABORTED
    
    def state_completed(self):
        """Mission completed successfully"""
        self.get_logger().info('STATE: COMPLETED - Mission successful!')
        # Generate report, save data, etc.
    
    def state_aborted(self):
        """Mission aborted or failed"""
        self.get_logger().error('STATE: ABORTED - Mission terminated')
        # Emergency procedures, RTH, save logs, etc.
    
    def publish_status(self):
        """Publish current status and save for visualization"""
        status_msg = String()
        status_msg.data = f'{self.state.name}|{self.mission_id}|errors:{self.error_count}'
        self.status_pub.publish(status_msg)
        
        # Save state to JSON file for web visualization
        state_data = {
            'state': self.state.name,
            'mission_id': self.mission_id,
            'error_count': self.error_count,
            'timestamp': time.time()
        }
        try:
            with open(self.viz_file, 'w') as f:
                json.dump(state_data, f)
        except Exception as e:
            self.get_logger().warning(f'Failed to write viz file: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    orchestrator = SimpleOrchestrator()
    
    try:
        orchestrator.run()
    except KeyboardInterrupt:
        pass
    finally:
        orchestrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
