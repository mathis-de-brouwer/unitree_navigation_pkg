import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory

class ROS2YOLOSegmenter(Node):
    def __init__(self):
        super().__init__('ros2_yolo_segmenter')

        # Get package directory to locate the model
        pkg_dir = get_package_share_directory('unitree_navigation_pkg')
        model_path = os.path.join(pkg_dir, 'models', 'botopiaRealCam.pt')
        
        self.get_logger().info(f'Loading model from: {model_path}')

        # Load YOLOv8 segmentation model
        self.model = YOLO(model_path, verbose=True)

        # Define colors per class (BGR format)
        self.classColors = {    
            'gangpad': (0, 255, 0),       # Green instead of Gray
            # Add any classes from botopiaRealCam.pt model
        }

        # Video capture setup (0 for webcam or provide video file path)
        self.cap = cv2.VideoCapture(0)
        self.frame_size = (480, 640)
        self.fps = 30
        self.prev_time = time.time()

        # ROS image publisher
        self.publisher = self.create_publisher(Image, '/segmentation/image', 10)
        self.bridge = CvBridge()

        # Timer callback
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.process_frame)

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame from camera.")
            return

        # Rotate image 90 degrees clockwise
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        
        # Resize after rotation
        frame = cv2.resize(frame, self.frame_size)

        # Inference
        inference_start = time.time()
        results = self.model(frame, conf=0.2, verbose=False)
        inference_end = time.time()
        inference_time_ms = (inference_end - inference_start) * 1000

        masked_frame = np.copy(frame)

        for result in results:
            if result.masks is not None and result.boxes is not None:
                for i, mask_xy in enumerate(result.masks.xy):
                    class_id = int(result.boxes.cls[i].item())
                    class_name = self.model.names[class_id]
                    conf = result.boxes.conf[i].item()  # Get confidence score

                    if class_name in self.classColors:
                        color = self.classColors[class_name]
                        points = np.array(mask_xy, dtype=np.int32)
                        cv2.fillPoly(masked_frame, [points], color=color)
                        
                        # Draw outline around mask
                        cv2.polylines(masked_frame, [points], isClosed=True, color=(255, 255, 255), thickness=2)
                        
                        # Display class name and confidence percentage
                        if points.shape[0] > 0:
                            text_pos = tuple(points[0])  # Position at first point of the polygon
                            
                            # Format text to include class name and confidence
                            conf_text = f"{class_name} {int(conf * 100)}%"
                            
                            # Add black background to text for better visibility
                            text_size = cv2.getTextSize(conf_text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)[0]
                            cv2.rectangle(
                                masked_frame, 
                                (text_pos[0], text_pos[1] - text_size[1] - 5),
                                (text_pos[0] + text_size[0], text_pos[1] + 5),
                                (0, 0, 0),
                                -1
                            )
                            
                            # Add text with white color
                            cv2.putText(
                                masked_frame,
                                conf_text,
                                text_pos,
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.8,
                                (255, 255, 255),  # White text
                                2,
                                lineType=cv2.LINE_AA
                            )

        # Publish to topic
        image_msg = self.bridge.cv2_to_imgmsg(masked_frame, encoding='bgr8')
        self.publisher.publish(image_msg)

        # Show FPS and inference time (only on display frame, not the published one)
        display_frame = masked_frame.copy()
        current_time = time.time()
        fps_text = 1 / (current_time - self.prev_time)
        self.prev_time = current_time

        cv2.putText(display_frame, f"FPS: {fps_text:.2f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.putText(display_frame, f"Inference: {inference_time_ms:.1f} ms", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        cv2.imshow("Segmentation View", display_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Exit requested.")
            self.cleanup()

    def cleanup(self):
        self.cap.release()
        cv2.destroyAllWindows()
        rclpy.shutdown()
        exit()

def main(args=None):
    rclpy.init(args=args)
    node = ROS2YOLOSegmenter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cleanup()
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()