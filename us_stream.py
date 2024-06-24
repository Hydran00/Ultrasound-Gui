import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import threading
from PyQt5.QtWidgets import QLabel, QHBoxLayout
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
import numpy as np
class USImageSubscriber:
    def __init__(self, layout):

        self.us_image_label = QLabel()
        self.us_image_label.setVisible(False)
        self.us_image_label.setMinimumSize(0, 0)
        layout.addWidget(self.us_image_label, 0, 3, 4, 1)

        image_subscriber = USImageSubscriberNode(self.us_image_label)
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(image_subscriber)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

class USImageSubscriberNode(Node):

    def __init__(self, us_image_label):
        super().__init__('gui_us_image_subscriber')
        self.us_image_label = us_image_label
        self.subscription = self.create_subscription(
            CompressedImage,
            'screen',
            self.listener_callback,
            10)
        self.br = CvBridge()
        self.get_logger().info('US Image Subscriber Node started')
        self.stream_active = False
        # create Qtimer that disable the label
        self.timer = QTimer()
        self.timer.timeout.connect(self.hide_us_image_label)
        self.timer.setSingleShot(False)
        self.timer.start(500)
        self.last_time = self.get_clock().now()

    def listener_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # to rgb
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        # Display the image
        h, w, ch = cv_image.shape
        bytesPerLine = ch * w
        convertToQtFormat = QImage(cv_image.data, w, h, bytesPerLine, QImage.Format_RGB888)
        
        pixmap = QPixmap(convertToQtFormat)
        scaled_pixmap = pixmap.scaled(self.us_image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.us_image_label.setPixmap(scaled_pixmap)
        self.us_image_label.setVisible(True)
        self.last_time = self.get_clock().now()

    def hide_us_image_label(self):
        # if no new image is received in 2 seconds, hide the label
        if (self.get_clock().now() - self.last_time).nanoseconds / 1e9 > 1:
            self.us_image_label.setVisible(False)
