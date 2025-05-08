#!/usr/bin/env python3
# ros2_websocket_bridge.py
# A ROS2 node that acts as a WebSocket server and bridges data to ROS2 topics

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import String, Float64MultiArray, Header
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import asyncio
import websockets
import json
import threading
import uuid
import signal
import sys
import traceback

class WebSocketBridgeNode(Node):
    def __init__(self):
        super().__init__('websocket_bridge_node')
        
        # Declare parameters
        self.declare_parameter('ws_port', 8069)
        self.declare_parameter('map_frame', 'map')
        
        # Get parameters
        self.ws_port = self.get_parameter('ws_port').get_parameter_value().integer_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        
        # Create QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create publishers
        self.path_pub = self.create_publisher(Path, '/navigation/global_path', qos_profile)
        self.waypoints_pub = self.create_publisher(Float64MultiArray, '/navigation/waypoints', qos_profile)
        self.status_pub = self.create_publisher(String, '/navigation/status', qos_profile)
        self.destination_pub = self.create_publisher(PoseStamped, '/navigation/destination', qos_profile)
        
        # WebSocket server
        self.websocket_server = None
        self.active_connections = set()
        
        # Start WebSocket server in a separate thread
        self.server_thread = threading.Thread(target=self.run_server)
        self.server_thread.daemon = True
        self.server_thread.start()
        
        # Set up signal handling for clean shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        
        self.get_logger().info(f'WebSocket Bridge Node initialized on port {self.ws_port}')
        self.publish_status("WebSocket server started and ready for connections")
    
    def signal_handler(self, sig, frame):
        """Handle shutdown signals"""
        self.get_logger().info('Shutdown signal received, closing WebSocket server...')
        if self.websocket_server:
            asyncio.run(self.stop_server())
        sys.exit(0)
    
    # FIXED: Removed the 'path' parameter to match websockets 10.x API
    async def websocket_handler(self, websocket):
        """Handle WebSocket connections and messages"""
        # Generate a unique client ID
        client_id = str(uuid.uuid4())
        
        # Get client info if available
        client_info = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}" if hasattr(websocket, 'remote_address') else client_id
        
        # Register the connection
        self.active_connections.add(websocket)
        self.get_logger().info(f'New client connected: {client_info}')
        self.publish_status(f"New client connected")
        
        try:
            # Send welcome message
            await websocket.send(json.dumps({
                'type': 'info',
                'message': 'Connected to ROS2 WebSocket Bridge'
            }))
            
            # Handle incoming messages
            async for message in websocket:
                try:
                    data = json.loads(message)
                    self.get_logger().info(f'Received message of type: {data.get("type", "unknown")}')
                    
                    if data.get('type') == 'directions':
                        # Process and publish the directions data
                        self.process_directions(data)
                        
                        # Send confirmation
                        await websocket.send(json.dumps({
                            'type': 'confirmation',
                            'message': 'Navigation data processed successfully'
                        }))
                    elif data.get('type') == 'ping':
                        # Respond to ping with pong
                        await websocket.send(json.dumps({
                            'type': 'pong',
                            'message': 'Server is alive'
                        }))
                    elif data.get('type') == 'hello':
                        # Respond to hello
                        await websocket.send(json.dumps({
                            'type': 'greeting',
                            'message': 'Hello from ROS2 WebSocket Bridge'
                        }))
                    else:
                        self.get_logger().warn(f'Unknown message type: {data.get("type", "unknown")}')
                        await websocket.send(json.dumps({
                            'type': 'error',
                            'message': f'Unknown message type: {data.get("type", "unknown")}'
                        }))
                
                except json.JSONDecodeError:
                    self.get_logger().error('Received invalid JSON data')
                    await websocket.send(json.dumps({
                        'type': 'error',
                        'message': 'Invalid JSON format'
                    }))
                except Exception as e:
                    self.get_logger().error(f'Error processing message: {str(e)}')
                    self.get_logger().error(traceback.format_exc())
                    await websocket.send(json.dumps({
                        'type': 'error',
                        'message': f'Server error: {str(e)}'
                    }))
        
        except websockets.exceptions.ConnectionClosed as e:
            self.get_logger().info(f'Client disconnected: {client_info} with code {e.code}')
        
        finally:
            # Unregister the connection
            if websocket in self.active_connections:
                self.active_connections.remove(websocket)
            self.publish_status(f"Client disconnected: {client_info}")
    
    def run_server(self):
        """Run the WebSocket server in a separate thread"""
        async def start_server():
            try:
                # Create an async event loop
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                
                self.get_logger().info(f"Starting WebSocket server on port {self.ws_port}")
                self.websocket_server = await websockets.serve(
                    self.websocket_handler, '0.0.0.0', self.ws_port,
                    # Add ping interval to keep connections alive
                    ping_interval=30,
                    ping_timeout=10
                )
                self.get_logger().info(f"WebSocket server started successfully on port {self.ws_port}")
                
                await self.websocket_server.wait_closed()
            except Exception as e:
                self.get_logger().error(f"Error in WebSocket server: {str(e)}")
                self.get_logger().error(traceback.format_exc())
        
        try:
            asyncio.run(start_server())
        except Exception as e:
            self.get_logger().error(f"Failed to run WebSocket server: {str(e)}")
            self.get_logger().error(traceback.format_exc())
    
    async def stop_server(self):
        """Stop the WebSocket server gracefully"""
        if self.websocket_server:
            self.websocket_server.close()
            await self.websocket_server.wait_closed()
    
    def process_directions(self, data):
        """Process the directions data and publish to ROS2 topics"""
        try:
            # Extract data
            route_coordinates = data.get('route', {}).get('coordinates', [])
            origin = data.get('origin', {})
            destination = data.get('destination', {})
            
            if not route_coordinates:
                self.get_logger().warn('No coordinates in route data')
                return
            
            # Get current ROS time
            current_time = self.get_clock().now().to_msg()
            
            # 1. Create and publish Path message
            path_msg = Path()
            path_msg.header = Header()
            path_msg.header.stamp = current_time
            path_msg.header.frame_id = self.map_frame
            
            for coord in route_coordinates:
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = coord[0]  # longitude
                pose.pose.position.y = coord[1]  # latitude
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0  # Identity quaternion
                path_msg.poses.append(pose)
            
            self.path_pub.publish(path_msg)
            
            # 2. Create and publish waypoints as Float64MultiArray
            # This format can be easier to work with for some applications
            waypoints_msg = Float64MultiArray()
            waypoints_data = []
            for coord in route_coordinates:
                waypoints_data.extend([coord[0], coord[1]])  # flatten to [x1, y1, x2, y2, ...]
            waypoints_msg.data = waypoints_data
            self.waypoints_pub.publish(waypoints_msg)
            
            # 3. Create and publish destination pose
            if destination:
                dest_msg = PoseStamped()
                dest_msg.header = path_msg.header
                dest_msg.pose.position.x = destination.get('lng', 0.0)
                dest_msg.pose.position.y = destination.get('lat', 0.0)
                dest_msg.pose.position.z = 0.0
                dest_msg.pose.orientation.w = 1.0
                self.destination_pub.publish(dest_msg)
            
            # Log success
            waypoint_count = len(route_coordinates)
            self.get_logger().info(f'Published navigation data with {waypoint_count} waypoints')
            self.publish_status(f"Received route with {waypoint_count} waypoints")
            
        except Exception as e:
            self.get_logger().error(f'Error processing directions: {str(e)}')
            self.get_logger().error(traceback.format_exc())
            self.publish_status(f"Error processing navigation data: {str(e)}")
    
    def publish_status(self, status_text):
        """Publish node status updates"""
        msg = String()
        msg.data = status_text
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WebSocketBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nReceived keyboard interrupt, shutting down...")
    except Exception as e:
        print(f"Unexpected error: {str(e)}")
        traceback.print_exc()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()