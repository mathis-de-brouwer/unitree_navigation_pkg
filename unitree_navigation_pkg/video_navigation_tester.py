import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import time
import argparse
import os
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO  # Import YOLO

class VideoNavTester(Node):
    def __init__(self, video_path=None):
        super().__init__('video_nav_tester')
        
        # Get video path from parameter if not provided directly
        if not video_path:
            self.declare_parameter('video_path', '')
            video_path = self.get_parameter('video_path').get_parameter_value().string_value
            if not video_path:
                pkg_dir = get_package_share_directory('unitree_navigation_pkg')
                video_path = os.path.join(pkg_dir, 'models', 'videobotopia.mp4')
                self.get_logger().info(f'Using default video path: {video_path}')
        
        # Load the YOLO model for segmentation
        pkg_dir = get_package_share_directory('unitree_navigation_pkg')
        model_path = os.path.join(pkg_dir, 'models', 'botopiaRealCam.pt')
        self.get_logger().info(f'Loading model from: {model_path}')
        
        # Load YOLO model
        try:
            self.model = YOLO(model_path, verbose=True)
            self.get_logger().info(f'Model loaded successfully')
            # Get the class names from the model
            self.class_names = self.model.names
            self.get_logger().info(f'Model classes: {self.class_names}')
        except Exception as e:
            self.get_logger().error(f'Error loading model: {e}')
            raise
        
        # Define colors per class (BGR format)
        self.classColors = {}
        
        # Define street/path related class names that should be green
        self.path_classes = [
            'gangpad', 'path', 'road', 'street', 'sidewalk', 'walkway', 'trail',
            'pavement', 'footpath', 'lane', 'driveway', 'alley', 'byway',
            'track', 'asphalt', 'concrete', 'paved', 'gravel'
        ]
        
        # Setup colors for all classes in the model
        for class_id, class_name in self.model.names.items():
            class_name_lower = class_name.lower()
            
            # Check if this class is related to paths/streets
            is_path_class = False
            for path_class in self.path_classes:
                if path_class in class_name_lower:
                    is_path_class = True
                    break
            
            if is_path_class:
                # All path/street classes are green
                self.classColors[class_name] = (0, 255, 0)  # Green in BGR
            else:
                # Other classes get random colors
                self.classColors[class_name] = (
                    np.random.randint(0, 255),
                    np.random.randint(0, 255),
                    np.random.randint(0, 255)
                )
        
        self.get_logger().info(f'Path classes will be colored green: {self.path_classes}')
        
        self.bridge = CvBridge()
        
        # Publishers
        self.pub_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_seg = self.create_publisher(Image, 'segmentation/image', 10)
        
        cv2.namedWindow('Nav View', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Nav View', 640, 480)
        
        cv2.namedWindow('Segmentation View', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Segmentation View', 640, 480)
        
        self.get_logger().info(f'Attempting to open video file: {video_path}')
        
        # Open video file
        self.video = cv2.VideoCapture(video_path)
        if not self.video.isOpened():
            self.get_logger().error(f'Could not open video file: {video_path}')
            raise RuntimeError(f'Could not open video file: {video_path}')
            
        self.fps = self.video.get(cv2.CAP_PROP_FPS)
        self.frame_size = (480, 640)  # Default size
        
        self.get_logger().info(f'Video opened successfully: {video_path}')
        self.get_logger().info(f'Video FPS: {self.fps}')
        
        # Navigation parameters
        self.declare_parameter('min_forward_speed', 0.1)
        self.declare_parameter('normal_speed', 0.2)
        self.declare_parameter('max_steering_angle', 0.5)
        self.declare_parameter('turning_speed_factor', 0.5)
        self.declare_parameter('playback_speed', 1.0)
        
        # Create a timer to process video frames
        playback_speed = self.get_parameter('playback_speed').value
        timer_period = 1.0 / (self.fps * playback_speed) if playback_speed > 0 else 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.process_frame)
        
        # Track processing times
        self.prev_time = time.time()
        
        self.get_logger().info('Video navigation tester initialized')

    def is_path_class(self, class_name):
        """Check if the class name is related to a path/road/street"""
        class_name_lower = class_name.lower()
        for path_class in self.path_classes:
            if path_class in class_name_lower:
                return True
        return False

    def process_frame(self):
        # Read frame from video
        ret, frame = self.video.read()
        if not ret:
            self.get_logger().info('End of video reached')
            self.timer.cancel()
            return
        
        # Resize frame if needed
        frame = cv2.resize(frame, self.frame_size)
        
        # Keep original frame for display
        display_frame = frame.copy()
        
        # Run inference using YOLO
        inference_start = time.time()
        results = self.model(frame, conf=0.25, verbose=False)
        inference_end = time.time()
        inference_time_ms = (inference_end - inference_start) * 1000
        
        # Create a segmentation visualization
        segmented_frame = frame.copy()
        
        # Create a mask to store segmented areas - specifically for paths
        h, w = frame.shape[:2]
        path_mask = np.zeros((h, w), dtype=np.uint8)
        
        # Process segmentation results
        for result in results:
            if result.masks is not None and result.boxes is not None:
                for i, mask_xy in enumerate(result.masks.xy):
                    class_id = int(result.boxes.cls[i].item())
                    class_name = self.model.names[class_id]
                    conf = result.boxes.conf[i].item()
                    
                    # Check if this is a path/street class
                    is_path = self.is_path_class(class_name)
                    
                    # Always use green for path classes, otherwise use the predefined color
                    color = (0, 255, 0) if is_path else self.classColors.get(class_name, (0, 0, 255))
                    
                    # Convert points to integer array
                    points = np.array(mask_xy, dtype=np.int32)
                    
                    # Fill the mask for this detection
                    cv2.fillPoly(segmented_frame, [points], color=color)
                    
                    # Also add to the binary path mask if it's a path/road class
                    if is_path:
                        cv2.fillPoly(path_mask, [points], color=255)
                    
                    # Draw outline around mask
                    cv2.polylines(segmented_frame, [points], isClosed=True, 
                                  color=(255, 255, 255), thickness=1)
                    
                    # Display class name and confidence
                    if points.shape[0] > 0:
                        text_pos = tuple(points[0])
                        conf_text = f"{class_name} {int(conf * 100)}%"
                        
                        # Add text background
                        text_size = cv2.getTextSize(conf_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)[0]
                        cv2.rectangle(
                            segmented_frame, 
                            (text_pos[0], text_pos[1] - text_size[1] - 5),
                            (text_pos[0] + text_size[0], text_pos[1] + 5),
                            (0, 0, 0),
                            -1
                        )
                        
                        # Add text
                        cv2.putText(
                            segmented_frame,
                            conf_text,
                            text_pos,
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (255, 255, 255),
                            1,
                            lineType=cv2.LINE_AA
                        )
        
        # Add timing information
        current_time = time.time()
        fps = 1 / (current_time - self.prev_time)
        self.prev_time = current_time
        
        cv2.putText(segmented_frame, f"FPS: {fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        cv2.putText(segmented_frame, f"Inference: {inference_time_ms:.1f} ms", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # Publish the segmented image for navigation node to use
        seg_msg = self.bridge.cv2_to_imgmsg(segmented_frame, encoding='bgr8')
        self.pub_seg.publish(seg_msg)
        
        # Display the original video frame and segmentation result
        cv2.imshow('Nav View', display_frame)
        cv2.imshow('Segmentation View', segmented_frame)
        
        # Create a visualization of the path mask
        path_vis = np.zeros_like(frame)
        path_vis[:,:,1] = path_mask  # Fill green channel with path mask
        
        # Display path mask in upper corner
        small_mask = cv2.resize(path_vis, (w//3, h//3))
        segmented_frame[0:h//3, 0:w//3] = small_mask
        
        cv2.waitKey(1)


def main(args=None):
    # Parse command line arguments for video path
    parser = argparse.ArgumentParser(description='Test navigation algorithm on a video file')
    parser.add_argument('--video_path', help='Path to the video file', default='')
    parsed_args, remaining = parser.parse_known_args()
    
    rclpy.init(args=args)
    node = VideoNavTester(parsed_args.video_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()