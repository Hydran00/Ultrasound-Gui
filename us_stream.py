import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
from PyQt5.QtWidgets import QLabel, QHBoxLayout
from PyQt5.QtCore import Qt
class USImageSubscriber:
    def __init__(self, layout):
        
        image_layout = QHBoxLayout()
        # ultrasound image feedback
        us_image = QLabel()
        image_layout.addWidget(us_image, Qt.AlignCenter)
        layout.addLayout(image_layout, 0, 3, 3, 1)

        image_subscriber = USImageSubscriberNode(image_layout)
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(image_subscriber)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

class USImageSubscriberNode(Node):

    def __init__(self, widget):
        super().__init__('gui_us_image_subscriber')
        self.widget = widget
        self.subscription = self.create_subscription(
            Image,
            'ultrasound_image',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.br = CvBridge()

    def listener_callback(self, msg):
        
        # Convert ROS Image message to OpenCV image
        cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Display the image
        h, w, ch = rgbImage.shape
        bytesPerLine = ch * w
        convertToQtFormat = QImage(rgbImage.data, w, h, bytesPerLine, QImage.Format_RGB888)
        pixmap = QPixmap(convertToQtFormat).scaled(w, h, QtCore.Qt.KeepAspectRatio)
        self.widget.setPixmap(pixmap)
