import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import os
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

class SimpleCameraBridge(Node):
    def __init__(self):
        super().__init__('simple_camera_bridge')
        
        # Declare parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('camera_id', 0)  # Use numeric ID instead of device path
        
        # Get parameters
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.camera_id = self.get_parameter('camera_id').get_parameter_value().integer_value
        
        if not model_path:
            pkg_dir = get_package_share_directory('unitree_navigation_pkg')
            model_path = os.path.join(pkg_dir, 'models', 'botopiaRealCam.pt')
        
        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        
        # Load YOLO model
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f'Model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Error loading model: {e}')
            raise
        
        # Define path-related classes
        self.path_classes = [
            'gangpad', 'path', 'road', 'street', 'sidewalk', 'walkway', 'trail',
            'pavement', 'footpath', 'lane', 'driveway', 'alley', 'byway',
            'track', 'asphalt', 'concrete', 'paved', 'gravel'
        ]
        
        # Initialize OpenCV capture
        self.get_logger().info(f'Opening camera with ID: {self.camera_id}')
        self.camera = cv2.VideoCapture(self.camera_id)
        
        if not self.camera.isOpened():
            self.get_logger().error(f'Could not open camera with ID: {self.camera_id}')
            raise RuntimeError(f'Could not open camera with ID: {self.camera_id}')
        
        # Create bridge and publishers
        self.bridge = CvBridge()
        self.seg_pub = self.create_publisher(Image, 'segmentation/image', 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Create display windows
        cv2.namedWindow('Camera Feed', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Camera Feed', 640, 480)
        
        cv2.namedWindow('Segmentation', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Segmentation', 640, 480)
        
        # Create timer for processing frames
        self.timer = self.create_timer(0.033, self.process_frame)  # ~30fps
        
        # Track performance
        self.prev_time = time.time()
        self.get_logger().info('Simple camera bridge initialized')
    
    def process_frame(self):
        """Process a frame from the camera"""
        ret, frame = self.camera.read()
        if not ret:
            self.get_logger().warn('Failed to read frame from camera')
            return
        
        # Keep original for display
        original = frame.copy()
        
        # Run YOLO model on frame
        inference_start = time.time()
        results = self.model(frame, conf=0.25, verbose=False)
        inference_end = time.time()
        inference_time_ms = (inference_end - inference_start) * 1000
        
        # Create visualization
        h, w = frame.shape[:2]
        path_mask = np.zeros((h, w), dtype=np.uint8)
        segmented = frame.copy()
        
        # Process results
        for result in results:
            if result.masks is not None and result.boxes is not None:
                for i, mask_xy in enumerate(result.masks.xy):
                    class_id = int(result.boxes.cls[i].item())
                    class_name = self.model.names[class_id]
                    conf = result.boxes.conf[i].item()
                    
                    # Check if path-related class
                    is_path = False
                    for path_class in self.path_classes:
                        if path_class in class_name.lower():
                            is_path = True
                            break
                    
                    # Set color based on class type
                    color = (0, 255, 0) if is_path else (0, 0, 255)
                    
                    # Convert points and draw mask
                    points = np.array(mask_xy, dtype=np.int32)
                    cv2.fillPoly(segmented, [points], color=color)
                    
                    # Add to path mask if it's a path
                    if is_path:
                        cv2.fillPoly(path_mask, [points], color=255)
                    
                    # Add outline
                    cv2.polylines(segmented, [points], True, (255, 255, 255), 1)
                    
                    # Add text label
                    if len(points) > 0:
                        pos = tuple(points[0])
                        text = f"{class_name} {int(conf * 100)}%"
                        
                        # Text background
                        text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
                        cv2.rectangle(
                            segmented,
                            (pos[0], pos[1] - text_size[1] - 5),
                            (pos[0] + text_size[0], pos[1] + 5),
                            (0, 0, 0),
                            -1
                        )
                        
                        # Text
                        cv2.putText(
                            segmented,
                            text,
                            pos,
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (255, 255, 255),
                            1,
                            cv2.LINE_AA
                        )
        
        # Calculate direction based on path mask
        bottom_third = path_mask[2*h//3:, :]
        left = np.sum(bottom_third[:, :w//3])
        center = np.sum(bottom_third[:, w//3:2*w//3])
        right = np.sum(bottom_third[:, 2*w//3:])
        
        # Create twist command
        twist = Twist()
        
        # Basic navigation logic
        if center > left and center > right:
            direction = "forward"
            twist.linear.x = 0.2
            twist.angular.z = 0.0
        elif left > right:
            direction = "left"
            twist.linear.x = 0.1
            twist.angular.z = 0.3
        else:
            direction = "right"
            twist.linear.x = 0.1
            twist.angular.z = -0.3
        
        # Add a small visualization of path mask
        path_vis = np.zeros_like(frame)
        path_vis[:,:,1] = path_mask  # Green channel
        small_mask = cv2.resize(path_vis, (w//3, h//3))
        segmented[0:h//3, 0:w//3] = small_mask
        
        # Add direction text and timing information
        current_time = time.time()
        fps = 1 / (current_time - self.prev_time)
        self.prev_time = current_time
        
        cv2.putText(segmented, f"Direction: {direction}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        cv2.putText(segmented, f"FPS: {fps:.1f}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        cv2.putText(segmented, f"Inference: {inference_time_ms:.1f} ms", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # Display frames
        cv2.imshow('Camera Feed', original)
        cv2.imshow('Segmentation', segmented)
        cv2.waitKey(1)
        
        # Publish segmented image and twist command
        try:
            seg_msg = self.bridge.cv2_to_imgmsg(segmented, encoding='bgr8')
            seg_msg.header.stamp = self.get_clock().now().to_msg()
            seg_msg.header.frame_id = 'camera'
            self.seg_pub.publish(seg_msg)
            self.cmd_pub.publish(twist)
        except Exception as e:
            self.get_logger().error(f'Error publishing data: {e}')
    
    def destroy_node(self):
        """Clean up resources when node shuts down"""
        if hasattr(self, 'camera') and self.camera is not None:
            self.camera.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    # Try different camera IDs if the default doesn't work
    try:
        node = SimpleCameraBridge()
        rclpy.spin(node)
    except RuntimeError as e:
        print(f"Failed with camera ID 0, trying ID 1: {e}")
        try:
            rclpy.shutdown()
            rclpy.init(args=args)
            
            # Try with camera ID 1
            node = Node('temp')
            node.declare_parameter('camera_id', 1)
            node.destroy_node()
            
            node = SimpleCameraBridge()
            rclpy.spin(node)
        except Exception as e2:
            print(f"Failed with all camera IDs: {e2}")
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()