#!/usr/bin/env python3
"""
Multi-Robot Control Server
Manages communication between robots and web clients

This server:
1. Receives sensor telemetry from robots
2. Broadcasts telemetry to connected web clients
3. Routes commands from web clients to specific robots
4. Maintains robot registry and status

Usage:
    python3 control_server.py [--port PORT] [--host HOST]
"""

import asyncio
import websockets
import json
import uuid
import logging
import sys
import threading
import os
import math
from datetime import datetime
from typing import Dict, Set, Optional, Any
from dataclasses import dataclass, asdict
from enum import Enum
from flask import Flask, request, jsonify, send_from_directory
from coverage_path_service import CoveragePathService

ALLOWED_HTTP_METHODS = 'GET, POST, OPTIONS'
ALLOWED_HTTP_HEADERS = 'Content-Type'

# ============================================================================
# Configuration
# ============================================================================

DEFAULT_PORT = 8765
DEFAULT_HOST = "0.0.0.0"
WORKSPACE_DIR = os.path.dirname(os.path.abspath(__file__))
HEARTBEAT_INTERVAL = 10  # seconds
ROBOT_TIMEOUT = 15  # seconds - robot considered offline if no data received

# ============================================================================
# Logging Setup
# ============================================================================

logging.basicConfig(
    level=logging.INFO,
    format='[%(levelname)s] %(asctime)s - %(name)s - %(message)s'
)
logger = logging.getLogger('ControlServer')

# ============================================================================
# Message Type Enum
# ============================================================================

class MessageType(Enum):
    # Robot-to-Server
    ROBOT_REGISTER = 0
    SENSOR_TELEMETRY = 1
    COMMAND_ACK = 2
    
    # Server-to-Robot
    COMMAND_MOVE = 10
    COMMAND_WAYPOINT = 11
    COMMAND_EMERGENCY_STOP = 12
    COMMAND_CANCEL_TASK = 13
    
    # Server-to-Web
    TELEMETRY_UPDATE = 20
    ROBOT_LIST = 21
    SERVER_STATUS = 22
    
    # Web-to-Server
    WEB_REGISTER = 30
    WEB_REQUEST_LIST = 31
    WEB_SEND_COMMAND = 32
    WEB_SELECT_ROBOT = 33

# ============================================================================
# Data Classes
# ============================================================================

@dataclass
class RobotInfo:
    robot_id: str
    robot_name: str
    robot_type: str
    max_speed: float
    online: bool = False
    last_update: int = 0
    battery_level: float = 100.0
    speed: float = 0.0
    heading: float = 0.0
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0

@dataclass
class WebSocketClient:
    client_id: str
    websocket: Any  # websockets.WebSocketServerProtocol
    client_type: str  # "robot" or "web"
    robot_id: Optional[str] = None
    robot_name: Optional[str] = None
    robot_type: Optional[str] = None
    max_speed: Optional[float] = None
    last_message: int = 0

# ============================================================================
# Control Server
# ============================================================================

class ControlServer:
    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT, http_port: int = 5000):
        self.host = host
        self.port = port
        self.http_port = http_port
        self.clients: Dict[str, WebSocketClient] = {}
        self.robots: Dict[str, RobotInfo] = {}
        self.server = None
        self.logger = logging.getLogger('ControlServer')
        self.coverage_service = CoveragePathService()
        
        # Setup Flask app for HTTP endpoints
        self.flask_app = Flask(__name__)
        self._setup_flask_routes()
        
    def _setup_flask_routes(self):
        """Setup Flask HTTP routes"""

        @self.flask_app.after_request
        def add_cors_headers(response):
            origin = request.headers.get('Origin', '*')
            response.headers['Access-Control-Allow-Origin'] = origin
            response.headers['Vary'] = 'Origin'
            response.headers['Access-Control-Allow-Methods'] = ALLOWED_HTTP_METHODS
            response.headers['Access-Control-Allow-Headers'] = ALLOWED_HTTP_HEADERS
            return response
        
        @self.flask_app.route('/api/coverage_plan', methods=['POST', 'OPTIONS'])
        def coverage_plan():
            """
            Generate coverage path
            
            POST /api/coverage_plan
            {
                "boundary": [[lat, lon], ...],
                "obstacles": [[[lat, lon], ...], ...],
                "row_spacing": 0.5,
                "safety_margin": 0.3,
                "sweep_angle": 0,
                "robot_speed": 1.0,
                "turn_threshold_deg": 8.0,
                "max_straight_spacing_m": 2.0,
                "max_waypoints": 200
            }
            
            Response:
            {
                "success": bool,
                "error": str or null,
                "waypoints": [{"lat": float, "lon": float, "heading": float}, ...],
                "cells_count": int,
                "total_distance": float,
                "estimated_time": float,
                "statistics": {...}
            }
            """
            try:
                if request.method == 'OPTIONS':
                    return ('', 204)

                data = request.get_json()
                
                # Validate required fields
                if not data or 'boundary' not in data:
                    return jsonify({
                        'success': False,
                        'error': 'Missing required field: boundary'
                    }), 400
                
                boundary = data.get('boundary', [])
                obstacles = data.get('obstacles', [])
                row_spacing = float(data.get('row_spacing', 0.5))
                safety_margin = float(data.get('safety_margin', 0.3))
                sweep_angle = float(data.get('sweep_angle', 0))
                robot_speed = float(data.get('robot_speed', 1.0))
                turn_threshold_deg = float(data.get('turn_threshold_deg', 8.0))
                max_straight_spacing_m = float(data.get('max_straight_spacing_m', 2.0))
                max_waypoints = int(data.get('max_waypoints', 200))
                
                # Generate coverage path
                result = self.coverage_service.plan_coverage(
                    boundary_latlon=boundary,
                    obstacles_latlon=obstacles,
                    row_spacing=row_spacing,
                    safety_margin=safety_margin,
                    sweep_angle=sweep_angle,
                    robot_speed=robot_speed,
                    turn_threshold_deg=turn_threshold_deg,
                    max_straight_spacing_m=max_straight_spacing_m,
                    max_waypoints=max_waypoints
                )
                
                return jsonify(result), 200 if result['success'] else 400
            
            except Exception as e:
                self.logger.error(f"Coverage plan error: {e}")
                return jsonify({
                    'success': False,
                    'error': str(e)
                }), 500

        @self.flask_app.route('/', methods=['GET'])
        def planner_index():
            return send_from_directory(WORKSPACE_DIR, 'mission_planner_v2.html')

        @self.flask_app.route('/mission_planner_v2.html', methods=['GET'])
        def planner_html():
            return send_from_directory(WORKSPACE_DIR, 'mission_planner_v2.html')
        
        @self.flask_app.route('/api/health', methods=['GET'])
        def health():
            """Health check endpoint"""
            return jsonify({
                'status': 'ok',
                'robots_online': sum(1 for r in self.robots.values() if r.online),
                'total_robots': len(self.robots)
            }), 200
        
        self.logger.info(f"Flask routes registered")
        
    def run_flask(self):
        """Run Flask app in a separate thread"""
        self.logger.info(f"Starting HTTP server on {self.host}:{self.http_port}")
        self.flask_app.run(host=self.host, port=self.http_port, debug=False, use_reloader=False)
        
    async def start(self):
        """Start the WebSocket server and Flask HTTP server"""
        self.logger.info(f"Starting Control Server on {self.host}:{self.port}")
        
        # Start Flask in background thread
        flask_thread = threading.Thread(target=self.run_flask, daemon=True)
        flask_thread.start()
        
        async with websockets.serve(self.handle_client, self.host, self.port):
            self.logger.info(f"Control Server listening on ws://{self.host}:{self.port}")
            self.logger.info(f"HTTP API listening on http://{self.host}:{self.http_port}")
            self.logger.info(f"Mission Planner UI available at http://127.0.0.1:{self.http_port}/mission_planner_v2.html")
            
            # Start background tasks
            await asyncio.gather(
                self.heartbeat_task(),
                self.robot_health_check_task()
            )
    
    async def handle_client(self, websocket):
        """Handle a new WebSocket client connection"""
        client_id = str(uuid.uuid4())[:8]
        client_type = None
        robot_id = None
        
        self.logger.info(f"Client connected: {client_id} from {websocket.remote_address}")
        
        try:
            async for message_str in websocket:
                try:
                    message = json.loads(message_str)
                    msg_type = message.get('type')
                    
                    # Route based on message type
                    if msg_type == MessageType.ROBOT_REGISTER.value:
                        client_type = "robot"
                        robot_id = message.get('robot_id')
                        await self.handle_robot_register(
                            client_id, robot_id, message, websocket
                        )
                    
                    elif msg_type == MessageType.WEB_REGISTER.value:
                        client_type = "web"
                        await self.handle_web_register(client_id, message, websocket)
                    
                    elif msg_type == MessageType.SENSOR_TELEMETRY.value:
                        await self.handle_telemetry(robot_id, message, websocket)
                    
                    elif msg_type == MessageType.WEB_SEND_COMMAND.value:
                        await self.handle_web_command(message, websocket)
                    
                    elif msg_type == MessageType.WEB_REQUEST_LIST.value:
                        await self.handle_robot_list_request(websocket)
                    
                    elif msg_type == MessageType.COMMAND_ACK.value:
                        payload = message.get('payload', {})
                        self.logger.debug(
                            f"[{robot_id}] Command ACK: {payload.get('command')} -> {payload.get('status')}"
                            + (f" ({payload.get('detail')})" if payload.get('detail') else "")
                        )
                    
                    else:
                        self.logger.warning(f"Unknown message type: {msg_type}")
                
                except json.JSONDecodeError as e:
                    self.logger.error(f"Invalid JSON from {client_id}: {e}")
                except Exception as e:
                    self.logger.error(f"Error handling message from {client_id}: {e}")
        
        except websockets.exceptions.ConnectionClosed:
            self.logger.info(f"Client disconnected: {client_id}")
        
        finally:
            # Cleanup
            if client_id in self.clients:
                del self.clients[client_id]
            
            if client_type == "robot" and robot_id in self.robots:
                self.robots[robot_id].online = False
                self.logger.info(f"Robot offline: {robot_id}")
                await self.broadcast_robot_list()
    
    async def handle_robot_register(self, client_id: str, robot_id: str,
                                   message: dict, websocket):
        """Handle robot registration"""
        payload = message.get('payload', {})
        
        robot_info = RobotInfo(
            robot_id=robot_id,
            robot_name=payload.get('robot_name', 'Unknown'),
            robot_type=payload.get('robot_type', 'unknown'),
            max_speed=payload.get('max_speed', 1.0),
            online=True,
            last_update=int(datetime.now().timestamp() * 1000)
        )
        
        # Register client
        self.clients[client_id] = WebSocketClient(
            client_id=client_id,
            websocket=websocket,
            client_type="robot",
            robot_id=robot_id,
            robot_name=robot_info.robot_name,
            robot_type=robot_info.robot_type,
            max_speed=robot_info.max_speed,
            last_message=int(datetime.now().timestamp())
        )
        
        # Register robot
        self.robots[robot_id] = robot_info
        
        self.logger.info(
            f"Robot registered: {robot_id} ({robot_info.robot_name}) - "
            f"Type: {robot_info.robot_type}, Speed: {robot_info.max_speed} m/s"
        )
        
        # Send ACK
        ack = {
            'type': MessageType.COMMAND_ACK.value,
            'robot_id': robot_id,
            'client_id': client_id,
            'payload': {'status': 'registered', 'server_version': '1.0'}
        }
        await websocket.send(json.dumps(ack))
        
        # Broadcast updated robot list to all web clients
        await self.broadcast_robot_list()
    
    async def handle_web_register(self, client_id: str, message: dict, websocket):
        """Handle web client registration"""
        payload = message.get('payload', {})
        
        self.clients[client_id] = WebSocketClient(
            client_id=client_id,
            websocket=websocket,
            client_type="web",
            last_message=int(datetime.now().timestamp())
        )
        
        self.logger.info(f"Web client registered: {client_id}")
        
        # Send ACK with current robot list
        ack = {
            'type': MessageType.COMMAND_ACK.value,
            'client_id': client_id,
            'payload': {'status': 'registered'}
        }
        await websocket.send(json.dumps(ack))
        
        # Send current robot list
        await self.broadcast_robot_list()
    
    async def handle_telemetry(self, robot_id: str, message: dict, websocket):
        """Handle sensor telemetry from robot"""
        if robot_id not in self.robots:
            self.logger.warning(f"Telemetry from unknown robot: {robot_id}")
            return
        
        payload = message.get('payload', {})
        status = payload.get('status', {})
        gps = payload.get('gps', {})
        
        # Update robot info
        robot = self.robots[robot_id]
        robot.online = True
        robot.last_update = int(datetime.now().timestamp() * 1000)
        robot.battery_level = status.get('battery_level', 100.0)
        robot.speed = status.get('speed', 0.0)
        robot.heading = status.get('heading', 0.0)
        gps_valid = bool(gps.get('valid', False))
        try:
            latitude = float(gps.get('latitude', 0.0))
            longitude = float(gps.get('longitude', 0.0))
            altitude = float(gps.get('altitude', 0.0))
            hdop = float(gps.get('hdop', 0.0))
        except (TypeError, ValueError):
            latitude = 0.0
            longitude = 0.0
            altitude = 0.0
            hdop = 0.0

        has_usable_fix = (
            gps_valid
            and math.isfinite(latitude)
            and math.isfinite(longitude)
            and abs(latitude) <= 90.0
            and abs(longitude) <= 180.0
            and (not math.isfinite(hdop) or hdop <= 5.0)
        )

        if has_usable_fix:
            robot.latitude = latitude
            robot.longitude = longitude
            robot.altitude = altitude
        
        # Update client last message time
        for client_id, client in list(self.clients.items()):
            if client.client_id == robot_id or (client.robot_id == robot_id):
                client.last_message = int(datetime.now().timestamp())
        
        # Broadcast to all web clients
        await self.broadcast_telemetry(message)
    
    async def handle_web_command(self, message: dict, websocket):
        """Handle command from web client"""
        robot_id = message.get('robot_id')
        payload = message.get('payload', {})
        command = payload.get('command')
        
        if robot_id not in self.robots:
            self.logger.error(f"Command to unknown robot: {robot_id}")
            return
        
        robot = self.robots[robot_id]
        if not robot.online:
            self.logger.error(f"Command to offline robot: {robot_id}")
            return
        
        self.logger.info(f"Routing {command} command to {robot_id}")
        
        # Convert web command to robot command type
        robot_command = self._create_robot_command(command, message)
        
        # Find robot connection and send command
        for client_id, client in list(self.clients.items()):
            if client.robot_id == robot_id and client.client_type == "robot":
                try:
                    await client.websocket.send(json.dumps(robot_command))
                except Exception as e:
                    self.logger.error(f"Failed to send command to {robot_id}: {e}")
                return

    @staticmethod
    def _sanitize_coverage_waypoints(waypoints):
        """Normalize and validate coverage waypoint list."""
        if not isinstance(waypoints, list):
            return []

        sanitized = []
        for wp in waypoints:
            if not isinstance(wp, dict):
                continue
            try:
                lat = float(wp.get('lat'))
                lon = float(wp.get('lon'))
            except (TypeError, ValueError):
                continue
            heading = wp.get('heading', 0.0)
            try:
                heading = float(heading)
            except (TypeError, ValueError):
                heading = 0.0
            sanitized.append({'lat': lat, 'lon': lon, 'heading': heading})
        return sanitized

    def _compile_mission_payload(self, payload: dict) -> dict:
        """Compile mission payload into firmware-ready waypoint-index tasks."""
        source = payload.get('source_mission') if isinstance(payload.get('source_mission'), dict) else None
        mission_name = payload.get('name', 'Unnamed Mission')
        mission_desc = payload.get('description', '')

        source_waypoints = source.get('waypoints', []) if source else payload.get('waypoints', [])
        source_tasks = source.get('tasks', []) if source else payload.get('tasks', [])

        source_coverage = source.get('coverage', {}) if source and isinstance(source.get('coverage'), dict) else payload.get('coverage', {})
        saved_paths = source_coverage.get('saved_paths', []) if isinstance(source_coverage, dict) else []

        runtime_waypoints = []
        for wp in source_waypoints:
            if not isinstance(wp, dict):
                continue
            try:
                lat = float(wp.get('lat', wp.get('latitude')))
                lon = float(wp.get('lon', wp.get('longitude')))
            except (TypeError, ValueError):
                continue
            alt = wp.get('alt', wp.get('altitude', 0.0))
            try:
                alt = float(alt)
            except (TypeError, ValueError):
                alt = 0.0
            runtime_waypoints.append({'lat': lat, 'lon': lon, 'alt': alt})

        runtime_tasks = []
        max_waypoints = 200

        for task in source_tasks:
            if not isinstance(task, dict):
                continue

            task_type = task.get('type', '')
            if task_type != 'coverage_path':
                runtime_tasks.append(dict(task))
                continue

            compiled_task = dict(task)
            wp_start = compiled_task.get('start_waypoint', compiled_task.get('waypoint_start'))
            wp_end = compiled_task.get('end_waypoint', compiled_task.get('waypoint_end'))

            has_range = isinstance(wp_start, int) and isinstance(wp_end, int)
            if has_range:
                runtime_tasks.append(compiled_task)
                continue

            coverage_points = self._sanitize_coverage_waypoints(compiled_task.get('coverage_waypoints', []))

            if not coverage_points:
                path_index = compiled_task.get('coverage_path_index')
                if isinstance(path_index, int) and 0 <= path_index < len(saved_paths):
                    path_obj = saved_paths[path_index] if isinstance(saved_paths[path_index], dict) else {}
                    coverage_points = self._sanitize_coverage_waypoints(path_obj.get('coverage_waypoints', []))
                    if not compiled_task.get('coverage_name'):
                        compiled_task['coverage_name'] = path_obj.get('name', 'Coverage')

            if not coverage_points:
                raise ValueError(f"Coverage task '{compiled_task.get('coverage_name', 'Coverage')}' has no usable waypoints")

            if len(runtime_waypoints) + len(coverage_points) > max_waypoints:
                raise ValueError(
                    f"Mission exceeds firmware waypoint limit ({max_waypoints}) while compiling coverage task '{compiled_task.get('coverage_name', 'Coverage')}'"
                )

            start_idx = len(runtime_waypoints)
            for wp in coverage_points:
                runtime_waypoints.append({'lat': wp['lat'], 'lon': wp['lon'], 'alt': 0.0})
            end_idx = len(runtime_waypoints) - 1

            compiled_task['start_waypoint'] = start_idx
            compiled_task['end_waypoint'] = end_idx
            compiled_task.pop('coverage_waypoints', None)

            runtime_tasks.append(compiled_task)

        if not runtime_waypoints:
            raise ValueError('Mission has no valid waypoints')

        return {
            'name': mission_name,
            'description': mission_desc,
            'waypoints': runtime_waypoints,
            'tasks': runtime_tasks
        }
    
    def _create_robot_command(self, command: str, web_message: dict) -> dict:
        """Convert web command to robot command message"""
        payload = web_message.get('payload', {})
        
        if command == 'move':
            return {
                'type': MessageType.COMMAND_MOVE.value,
                'robot_id': web_message.get('robot_id'),
                'client_id': web_message.get('client_id'),
                'payload': {
                    'command': 'move',
                    'linear_velocity': payload.get('linear_velocity', 0.0),
                    'angular_velocity': payload.get('angular_velocity', 0.0),
                    'duration': payload.get('duration', 5.0)
                }
            }
        elif command == 'waypoint':
            return {
                'type': MessageType.COMMAND_WAYPOINT.value,
                'robot_id': web_message.get('robot_id'),
                'client_id': web_message.get('client_id'),
                'payload': {
                    'command': 'waypoint',
                    'latitude': payload.get('latitude', 0.0),
                    'longitude': payload.get('longitude', 0.0),
                    'altitude': payload.get('altitude', 0.0)
                }
            }
        elif command == 'emergency_stop':
            return {
                'type': MessageType.COMMAND_EMERGENCY_STOP.value,
                'robot_id': web_message.get('robot_id'),
                'client_id': web_message.get('client_id'),
                'payload': {
                    'command': 'emergency_stop'
                }
            }
        elif command == 'execute_mission':
            compiled = self._compile_mission_payload(payload)
            # Send firmware-ready mission while preserving compatibility with source mission references.
            return {
                'type': 14,  # COMMAND_EXECUTE_MISSION (will be 14)
                'robot_id': web_message.get('robot_id'),
                'client_id': web_message.get('client_id'),
                'payload': {
                    'name': compiled.get('name', 'Unnamed Mission'),
                    'description': compiled.get('description', ''),
                    'waypoints': compiled.get('waypoints', []),
                    'tasks': compiled.get('tasks', [])
                }
            }
        elif command == 'pause_mission':
            # Pause mission command
            return {
                'type': 15,  # COMMAND_PAUSE_MISSION
                'robot_id': web_message.get('robot_id'),
                'client_id': web_message.get('client_id'),
                'payload': {
                    'command': 'pause_mission'
                }
            }
        elif command == 'cancel_mission':
            # Cancel mission command
            return {
                'type': 16,  # COMMAND_CANCEL_MISSION
                'robot_id': web_message.get('robot_id'),
                'client_id': web_message.get('client_id'),
                'payload': {
                    'command': 'cancel_mission'
                }
            }
        else:
            self.logger.warning(f"Unknown command type: {command}")
            return web_message
    
    async def handle_robot_list_request(self, websocket):
        """Handle request for robot list from web client"""
        await self.broadcast_robot_list(target_websocket=websocket)
    
    async def broadcast_robot_list(self, target_websocket=None):
        """Broadcast robot list to all (or specific) web clients"""
        robots_data = []
        for robot_id, robot in list(self.robots.items()):
            robots_data.append({
                'robot_id': robot.robot_id,
                'robot_name': robot.robot_name,
                'robot_type': robot.robot_type,
                'max_speed': robot.max_speed,
                'online': robot.online,
                'last_update': robot.last_update,
                'battery_level': robot.battery_level,
                'speed': robot.speed,
                'heading': robot.heading,
                'latitude': robot.latitude,
                'longitude': robot.longitude,
                'altitude': robot.altitude
            })
        
        message = {
            'type': MessageType.ROBOT_LIST.value,
            'timestamp': int(datetime.now().timestamp() * 1000),
            'payload': {'robots': robots_data}
        }
        
        message_json = json.dumps(message)
        
        if target_websocket:
            try:
                await target_websocket.send(message_json)
            except Exception as e:
                self.logger.error(f"Failed to send robot list: {e}")
        else:
            # Send to all web clients
            disconnected = []
            for client_id, client in list(self.clients.items()):
                if client.client_type == "web":
                    try:
                        await client.websocket.send(message_json)
                    except Exception as e:
                        self.logger.error(f"Failed to send robot list to {client_id}: {e}")
                        disconnected.append(client_id)
            
            for client_id in disconnected:
                self.clients.pop(client_id, None)
    
    async def broadcast_telemetry(self, telemetry_message: dict):
        """Broadcast telemetry to all web clients"""
        message_json = json.dumps(telemetry_message)
        
        disconnected = []
        for client_id, client in list(self.clients.items()):
            if client.client_type == "web":
                try:
                    await client.websocket.send(message_json)
                except Exception as e:
                    self.logger.error(f"Failed to send telemetry to {client_id}: {e}")
                    disconnected.append(client_id)
        
        for client_id in disconnected:
            self.clients.pop(client_id, None)
    
    async def heartbeat_task(self):
        """Periodic heartbeat to keep connections alive"""
        while True:
            await asyncio.sleep(HEARTBEAT_INTERVAL)
            
            message = {
                'type': 99,  # Heartbeat type
                'timestamp': int(datetime.now().timestamp() * 1000)
            }
            message_json = json.dumps(message)
            
            disconnected = []
            for client_id, client in list(self.clients.items()):
                try:
                    await client.websocket.send(message_json)
                except Exception as e:
                    self.logger.debug(f"Failed to send heartbeat to {client_id}: {e}")
                    disconnected.append(client_id)
            
            for client_id in disconnected:
                self.clients.pop(client_id, None)
    
    async def robot_health_check_task(self):
        """Check robot health and mark offline if no recent telemetry"""
        while True:
            await asyncio.sleep(5)  # Check every 5 seconds
            
            now = int(datetime.now().timestamp() * 1000)
            timeout_ms = ROBOT_TIMEOUT * 1000
            
            for robot_id, robot in list(self.robots.items()):
                if robot.online and (now - robot.last_update) > timeout_ms:
                    self.logger.warning(f"Robot timeout: {robot_id}")
                    robot.online = False
                    await self.broadcast_robot_list()

# ============================================================================
# Main
# ============================================================================

async def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Multi-Robot Control Server')
    parser.add_argument('--host', default=DEFAULT_HOST, help=f'Host to bind to (default: {DEFAULT_HOST})')
    parser.add_argument('--port', type=int, default=DEFAULT_PORT, help=f'WebSocket port to bind to (default: {DEFAULT_PORT})')
    parser.add_argument('--http-port', type=int, default=5000, help=f'HTTP API port to bind to (default: 5000)')
    
    args = parser.parse_args()
    
    server = ControlServer(host=args.host, port=args.port, http_port=args.http_port)
    
    try:
        await server.start()
    except KeyboardInterrupt:
        logger.info("Shutting down...")

if __name__ == '__main__':
    asyncio.run(main())
