import subprocess
import sys, os
import cv2
from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QGridLayout, QPlainTextEdit, QLabel, QSizePolicy, QLineEdit, QListWidget, QVBoxLayout, QHBoxLayout, QCheckBox
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

# { 'param name' , ['default idx', ['options']] }
ARGS= {
    "use_ft_sensor": [1,[True,False]], # True or False
    "camera_type": [0,["realsense","zed"]], # realsense or zed
    "robot_type": [0,["kuka","ur3e"]], # kuka or ur3e
    "video_stream":  [0,[True,False]], # True or False
    "point_cloud_stream": [0,[True,False]], # True or False
    "encode_streams": [0,[True,False]], # True or False
}


class MainWindow(QWidget):

    def __init__(self):
        super().__init__()
        self.bounding_box = int(0)
        self.scaling_factor = float(1)

        self.setWindowTitle("Control Panel")
        self.setWindowIcon(QtGui.QIcon('assets/logo.png'))
        # set size
        self.setMinimumSize(1000, 800)
        app_layout = QVBoxLayout()
        self.setLayout(app_layout)
        # self.setWindowFlags(Qt.WindowStaysOnTopHint)

        # Apply the PyQTDark theme
        app.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())

        # Create QLabel to display the current frame
        self.frame_label = QLabel()
        # Start the QTimer to update the frame
        self.timer = QTimer(self)
        # self.timer.timeout.connect(self.update_frame)
        self.timer.start(40)  # Update every 40 milliseconds (25 frames per second)

        # Dictionary to hold buttons, text boxes, and scroll buttons
        self.button_textbox_map = {}

        # divide app layout into two rows (first options, second for buttons and text boxes)
        opt_layout = QHBoxLayout()
        layout = QGridLayout()

        # add options
        use_ft_sensor_layout = utils.create_checkbox_options_layout("Use FT Sensor")
        camera_selection_layout = utils.create_combo_options_layout("Select the camera:", ARGS["camera_type"][1], ARGS["camera_type"][0])
        robot_selection_layout = utils.create_combo_options_layout("Select the robot:", ARGS["robot_type"][1], ARGS["robot_type"][0])
        video_stream_layout = utils.create_checkbox_options_layout("Video Stream")
        point_cloud_stream_layout = utils.create_checkbox_options_layout("Point Cloud Stream")
        encode_streams_layout = utils.create_checkbox_options_layout("Encode Streams")

        # Add point cloud stream option
        point_cloud_stream_layout = QVBoxLayout()
        label = QLabel()
        label.setText("Point Cloud Stream:")
        label.setAlignment(Qt.AlignLeft)
        label.setFixedSize(200, 45)
        point_cloud_stream_layout.addWidget(label)
        self.point_cloud_stream_switch = QCheckBox()
        # set default

        # add to layout
        opt_layout.addLayout(use_ft_sensor_layout, Qt.AlignCenter)          
        opt_layout.addLayout(camera_selection_layout, Qt.AlignCenter)
        opt_layout.addLayout(robot_selection_layout, Qt.AlignCenter)
        opt_layout.addLayout(video_stream_layout, Qt.AlignCenter)
        opt_layout.addLayout(point_cloud_stream_layout, Qt.AlignCenter)
        opt_layout.addLayout(encode_streams_layout, Qt.AlignCenter)

        # add options
        app_layout.addLayout(opt_layout)


        # add buttons, text boxes, and scroll buttons
        commands = utils.read_from_file("assets/commands.txt")
        labels = utils.read_from_file("assets/labels.txt")
        text_box_command_layouts = []
        for i, command in enumerate(commands):
            output_textbox = QPlainTextEdit()
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
            
            # if "follower" in command:


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
        # self.us_image_feedback = USImageSubscriber(layout)
        app_layout.addLayout(layout)  
            
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
