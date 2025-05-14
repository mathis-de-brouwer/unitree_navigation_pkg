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
        
        # Simple navigation parameters
        self.declare_parameter('constant_speed', 0.2)  # Constant forward speed
        self.declare_parameter('turning_angle', 0.4)   # Fixed turning angle
        
        # State variables for intersection handling
        self.at_intersection = False
        self.last_intersection_time = time.time()
        self.intersection_cooldown = 5.0  # seconds
        
        self.get_logger().info('Navigation node initialized')
        
    def cb_seg(self, msg: Image):
        # Convert ROS Image → OpenCV BGR
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Create mask for green areas (path detection)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Green color range in HSV
        lower_green = np.array([40, 100, 100])
        upper_green = np.array([80, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # Get image dimensions
        h, w = mask.shape
        
        # Split the bottom portion of the image into left/center/right
        # Use bottom third for better near-field detection
        bottom_third = mask[2*h//3:, :]
        thirds = np.array_split(bottom_third, 3, axis=1)
        
        left_count = np.sum(thirds[0])
        center_count = np.sum(thirds[1])
        right_count = np.sum(thirds[2])
        
        # Create Twist message with constant speed
        twist = Twist()
        twist.linear.x = self.get_parameter('constant_speed').value
        
        # Simple direction determination
        # If center has the most white pixels, go straight
        # Otherwise turn toward the side with more white pixels
        direction = ""
        turning_angle = self.get_parameter('turning_angle').value
        
        if center_count > left_count and center_count > right_count:
            # Center has most path - go straight
            twist.angular.z = 0.0
            direction = "CENTER"
        elif left_count > right_count:
            # Left has more path - turn left
            twist.angular.z = turning_angle
            direction = "LEFT"
        else:
            # Right has more path - turn right
            twist.angular.z = -turning_angle
            direction = "RIGHT"
        
        # Display the direction on frame
        cv2.putText(frame, f"Direction: {direction}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        # Draw simple direction arrow
        arrow_start = (w//2, h-20)  # bottom-center
        if direction == "CENTER":
            arrow_end = (w//2, h//2)  # straight up
        elif direction == "LEFT":
            arrow_end = (w//4, h//2)  # up-left
        else:
            arrow_end = (3*w//4, h//2)  # up-right
            
        cv2.arrowedLine(frame, arrow_start, arrow_end, 
                        (0, 0, 255), thickness=3, tipLength=0.3)
        
        # Display the path mask for debugging (small window)
        mask_small = cv2.resize(cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), (w//3, h//3))
        frame[0:h//3, 0:w//3] = mask_small
        
        # Publish the twist command
        self.pub_vel.publish(twist)
        
        # Show the navigation view
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
