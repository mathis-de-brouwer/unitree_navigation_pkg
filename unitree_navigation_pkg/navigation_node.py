import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist
import time

class NavVisualizer(Node):
    def __init__(self):
        super().__init__('nav_visualizer')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, 'segmentation/image', self.cb_seg, 10)
        self.pub_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        cv2.namedWindow('Nav View', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Nav View', 640, 480)
        
        # Navigation parameters
        self.declare_parameter('min_forward_speed', 0.1)
        self.declare_parameter('normal_speed', 0.2)
        self.declare_parameter('max_steering_angle', 0.5)
        self.declare_parameter('turning_speed_factor', 0.5)
        
        # State variables for intersection handling and dead-end detection
        self.is_turning_around = False
        self.turn_around_start_time = None
        self.turn_around_duration = 3.0  # seconds to complete 180-degree turn
        self.at_intersection = False
        self.last_intersection_time = self.get_clock().now()
        self.intersection_cooldown = 5.0  # seconds
        
        self.get_logger().info('Navigation node initialized')

    def cb_seg(self, msg: Image):
        # Convert ROS Image → OpenCV BGR
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Create mask for green areas (path detection from gangpad model)
        # Looking for close to pure green (0,255,0) with some tolerance
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Green color range in HSV
        lower_green = np.array([40, 100, 100])
        upper_green = np.array([80, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # Get image dimensions
        h, w = mask.shape
        
        # Check for dead end (very little path visible)
        path_coverage = np.sum(mask) / (h * w * 255)
        if path_coverage < 0.05 and not self.is_turning_around:
            self.is_turning_around = True
            self.turn_around_start_time = time.time()
            self.get_logger().info('Dead end detected! Turning around...')
            
            # Execute turn-around
            twist = Twist()
            twist.linear.x = 0.1
            twist.angular.z = 0.8  # Strong turn
            self.pub_vel.publish(twist)
            
            # Draw feedback on the frame
            cv2.putText(frame, "TURNING AROUND", (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            cv2.imshow('Nav View', frame)
            cv2.waitKey(1)
            return
            
        # Check if still in turn-around maneuver
        if self.is_turning_around:
            if time.time() - self.turn_around_start_time < self.turn_around_duration:
                twist = Twist()
                twist.linear.x = 0.1
                twist.angular.z = 0.8  # Continue turning
                self.pub_vel.publish(twist)
                
                cv2.putText(frame, "TURNING AROUND", (10, 60), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                cv2.imshow('Nav View', frame)
                cv2.waitKey(1)
                return
            else:
                self.is_turning_around = False
                self.get_logger().info('Turn around completed.')
        
        # Check for intersection pattern
        vertical_thirds = np.array_split(mask, 3, axis=0)
        bottom_third = vertical_thirds[2]
        horizontal_thirds = np.array_split(bottom_third, 3, axis=1)
        
        left_count = np.sum(horizontal_thirds[0])
        center_count = np.sum(horizontal_thirds[1])
        right_count = np.sum(horizontal_thirds[2])
        
        # Detect Y-intersection: significant path in all thirds
        time_since_last = (time.time() - self.last_intersection_time 
                          if hasattr(self, 'last_intersection_time') else 999)
        
        if (left_count > 1000 and center_count > 1000 and right_count > 1000 and 
            time_since_last > self.intersection_cooldown and not self.at_intersection):
            self.at_intersection = True
            self.last_intersection_time = time.time()
            self.get_logger().info('Intersection detected!')
            
            # Navigate through intersection - choose randomly between left and right
            twist = Twist()
            twist.linear.x = 0.15  # Slow down at intersection
            
            # Simple random choice at intersection
            if np.random.random() > 0.5:
                twist.angular.z = 0.3  # Turn right
                direction = "RIGHT"
            else:
                twist.angular.z = -0.3  # Turn left
                direction = "LEFT"
                
            self.pub_vel.publish(twist)
            
            # Draw intersection feedback
            cv2.putText(frame, f"INTERSECTION: {direction}", (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.imshow('Nav View', frame)
            cv2.waitKey(1)
            return
        elif self.at_intersection:
            # Check if we're still at the intersection
            if time.time() - self.last_intersection_time > 2.0:
                self.at_intersection = False
            else:
                # Continue with the previous turn
                cv2.putText(frame, "IN INTERSECTION", (10, 60), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.imshow('Nav View', frame)
                cv2.waitKey(1)
                return
                
        # Normal path following - split mask into left/center/right thirds
        thirds = np.array_split(mask, 3, axis=1)
        counts = [np.sum(t) for t in thirds]
        direction = int(np.argmax(counts))  # 0=left,1=center,2=right

        # Calculate path centroid for smoother steering
        M = cv2.moments(mask)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            # Calculate steering based on centroid position
            # Map cx from 0-w to -max_angle to +max_angle
            max_angle = self.get_parameter('max_steering_angle').value
            normalized_x = (cx / w) - 0.5  # Range: -0.5 to 0.5
            steering_angle = max_angle * (normalized_x * 2.0)
        else:
            # Fallback to direction-based steering if no centroid
            if direction == 1:
                steering_angle = 0.0
            elif direction == 0:
                steering_angle = self.get_parameter('max_steering_angle').value
            else:
                steering_angle = -self.get_parameter('max_steering_angle').value

        # Build and publish a Twist message with proportional control
        twist = Twist()
        normal_speed = self.get_parameter('normal_speed').value
        turning_factor = self.get_parameter('turning_speed_factor').value
        min_speed = self.get_parameter('min_forward_speed').value
        
        # Calculate speed based on steering angle (reduce speed in turns)
        speed_reduction = abs(steering_angle) / max_angle
        twist.linear.x = max(normal_speed * (1 - speed_reduction * turning_factor), min_speed)
        twist.angular.z = -steering_angle  # Negative because of coordinate system
        
        self.pub_vel.publish(twist)

        # Draw arrow on the BGR frame for visualization
        arrow_start = (w//2, h)             # bottom‐center
        if direction == 1:
            arrow_end = (w//2, h//3)        # straight up
            dir_text = "C"
        elif direction == 0:
            arrow_end = (w//4, h//2)        # up‐left
            dir_text = "L"
        else:
            arrow_end = (3*w//4, h//2)      # up‐right
            dir_text = "R"

        cv2.arrowedLine(frame, arrow_start, arrow_end,
                        (0,0,255), thickness=5, tipLength=0.3)

        cv2.putText(frame, f'Dir: {dir_text} Angle: {steering_angle:.2f} Speed: {twist.linear.x:.2f}',
                    (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

        # Display the path mask for debugging (small window)
        mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        mask_small = cv2.resize(mask_rgb, (w//3, h//3))
        frame[0:h//3, 0:w//3] = mask_small

        cv2.imshow('Nav View', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = NavVisualizer()
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
