import subprocess
import sys, os
import cv2
from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QGridLayout, QTextEdit, QLabel, QSizePolicy, QLineEdit, QListWidget, QVBoxLayout, QHBoxLayout, QCheckBox
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QPixmap
from PyQt5.QtCore import QProcess, pyqtSlot
from PyQt5.QtGui import QImage,QColor  
from PyQt5 import QtGui
import utils
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
import qdarkstyle  # Import the qdarkstyle library

from buttons.base import BaseButton
from buttons.motion_handle import HandleButton
from buttons.tcp_endpoint import TcpEndpointButton
from buttons.ft_calibration import FTCalibrationButton
from us_stream import USImageSubscriber
class MainWindow(QWidget):

    def __init__(self):
        super().__init__()
        self.bounding_box = int(0)
        self.scaling_factor = float(1)

        self.setWindowTitle("Control Panel")
        self.setWindowIcon(QtGui.QIcon('assets/logo.png'))
        # set size
        self.setMinimumSize(1000, 800)
        layout = QGridLayout()
        self.setLayout(layout)
        # self.setWindowFlags(Qt.WindowStaysOnTopHint)

        # Apply the PyQTDark theme
        app.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())

        # Create QLabel to display the current frame
        self.frame_label = QLabel()
        layout.addWidget(self.frame_label, 0, 2, -1, 1)  # Span all rows

        # Start the QTimer to update the frame
        self.timer = QTimer(self)
        # self.timer.timeout.connect(self.update_frame)
        self.timer.start(40)  # Update every 40 milliseconds (25 frames per second)

        # Dictionary to hold buttons, text boxes, and scroll buttons
        self.button_textbox_map = {}

        # add buttons, text boxes, and scroll buttons
        commands = utils.read_from_file("assets/commands.txt")
        labels = utils.read_from_file("assets/labels.txt")
        text_box_command_layouts = []
        for i, command in enumerate(commands):
            output_textbox = QTextEdit()
            output_textbox.setReadOnly(True)
            layout.addWidget(output_textbox, i, 1)

            text_box_command_layout = QVBoxLayout()

            # scroll_button = QCheckBox("AutoScroll")
            # scroll_button.setFixedSize(100, 30)
            # scroll_button.clicked.connect(lambda checked, tb=output_textbox, sb=scroll_button: self.toggle_autoscroll(sb, tb))
            # text_box_command_layout.addWidget(scroll_button, Qt.AlignCenter)
            
            clear_button = QPushButton("Clear")
            clear_button.setFixedSize(100, 30)
            clear_button.clicked.connect(lambda checked, tb=output_textbox: tb.clear())
            text_box_command_layout.addWidget(clear_button, Qt.AlignCenter)

            print("Adding button for command: ", command)
            
            if "ros_tcp_endpoint" in command:
                # Select the network interface to use
                net_selection_layout = QVBoxLayout()
                label = QLabel()
                label.setText("Select the network\n interface:")
                label.setAlignment(Qt.AlignLeft)
                label.setFixedSize(200, 45)
                net_selection_layout.addWidget(label)
                self.net_interfaces = utils.get_net_interfaces()
                self.net_interface_switch = QListWidget()
                self.net_interface_switch.addItems(self.net_interfaces)
                # set default interface to the first one
                self.net_interface_switch.setCurrentRow(0)
                self.net_interface_switch.setFixedSize(200, 100)
                # Create the button
                button_launch = TcpEndpointButton(command, labels[i], output_textbox, self.net_interface_switch)
                net_selection_layout.addWidget(self.net_interface_switch)
                text_box_command_layout.addLayout(net_selection_layout, Qt.AlignCenter)

            else:
                button_launch = BaseButton(command, labels[i], output_textbox)
            
            button_launch.setMinimumSize(150, 100)
            layout.addWidget(button_launch, i, 0)
            self.button_textbox_map[button_launch] = (output_textbox)
            text_box_command_layouts.append(text_box_command_layout)

            layout.addLayout(text_box_command_layout, i, 2)


        handle_layout = QHBoxLayout()
        # add interactive marker button
        button_handle = HandleButton()
        button_handle.setFixedSize(150, 100)
        handle_layout.addWidget(button_handle, Qt.AlignCenter)
        
        # add ft sensor calibration
        button_ft_calibration = FTCalibrationButton()
        button_ft_calibration.setFixedSize(150, 100)
        handle_layout.addWidget(button_ft_calibration, Qt.AlignCenter)
        
        layout.addLayout(handle_layout, len(commands), 0, 1, 2)
        # ultrasound image feedback
        self.us_image_feedback = USImageSubscriber(layout)  
            
    def resizeEvent(self, event):
        # Resize the pixmap when the window is resized
        us_image_label = self.us_image_feedback.us_image_label
        if us_image_label.pixmap() is not None:
            scaled_pixmap = us_image_label.pixmap().scaled(us_image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            us_image_label.setPixmap(scaled_pixmap)
        super().resizeEvent(event)
        
if __name__ == "__main__":
    rclpy.init(args=None)
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    try:
        sys.exit(app.exec_())
    finally:
        # Clean up when the application is closed
        rclpy.shutdown()
