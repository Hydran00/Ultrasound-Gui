import subprocess
import sys, os
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
        self.setWindowFlags(Qt.WindowStaysOnTopHint)

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

        # Button to toggle bounding_box parameter
        bounding_box_button = QPushButton("Toggle Bounding Box")
        bounding_box_button.setMinimumSize(150, 40)
        bounding_box_button.clicked.connect(lambda checked, ros_node=self: ros_node.toggle_bounding_box())
        layout.addWidget(bounding_box_button, len(commands) + 1, 0)

        # QLineEdit to display and edit the value of self.bounding_box
        self.bounding_box_edit = QLineEdit(str(self.bounding_box))

        self.bounding_box_edit.setFixedWidth(100)  # Set the width according to your preference
        self.bounding_box_edit.setReadOnly(True)   # Make it read-only
        layout.addWidget(self.bounding_box_edit, len(commands) + 1, 1)

        # Button to decrease scaling factor
        decrease_scaling_factor_button = QPushButton("Decrease Scaling Factor")
        decrease_scaling_factor_button.setMinimumSize(150, 40)
        decrease_scaling_factor_button.clicked.connect(lambda checked, ros_node=self: ros_node.decrease_scaling_factor())
        layout.addWidget(decrease_scaling_factor_button, len(commands) + 3, 0)

        # Button to increase scaling factor
        increase_scaling_factor_button = QPushButton("Increase Scaling Factor")
        increase_scaling_factor_button.setMinimumSize(150, 40)
        increase_scaling_factor_button.clicked.connect(lambda checked, ros_node=self: ros_node.increase_scaling_factor())
        layout.addWidget(increase_scaling_factor_button, len(commands) + 4, 0)

        # QLineEdit to display the value of self.scaling_factor
        self.scaling_factor_edit = QLineEdit(str(self.scaling_factor))
        self.scaling_factor_edit.setFixedWidth(100)  # Set the width according to your preference
        self.scaling_factor_edit.setReadOnly(True)   # Make it read-only
        layout.addWidget(self.scaling_factor_edit, len(commands) + 4, 1)

    def toggle_bounding_box(self):
        self.bounding_box = int(not self.bounding_box)
        command = ['ros2', 'param', 'set', 'haptic_control', 'bounding_box', str(self.bounding_box)]
        print(command)
        try:
            subprocess.run(command, check=True)
            print('Toggled bounding box to', bool(self.bounding_box))
            # Update the displayed value in the QLineEdit
            self.bounding_box_edit.setText(str(self.bounding_box))
        except subprocess.CalledProcessError as e:
            print('Error setting bounding box')

    def decrease_scaling_factor(self):
        self.scaling_factor = self.scaling_factor - 0.25 

        if self.scaling_factor < 0.0:
            self.scaling_factor = 0.0

        command = ['ros2', 'param', 'set', 'haptic_control', 'scaling_factor', str(self.scaling_factor)]
        print(command)
        try:
            subprocess.run(command, check=True)
            print('Setting scaling factor to', str(self.scaling_factor))
            # Update the displayed value in the QLineEdit
            self.scaling_factor_edit.setText(str(self.scaling_factor))
        except subprocess.CalledProcessError as e:
            print('Error setting scaling factor')

    def increase_scaling_factor(self):
        self.scaling_factor = self.scaling_factor + 0.25 

        if self.scaling_factor > 1.0:
            self.scaling_factor = 1.0

        command = ['ros2', 'param', 'set', 'haptic_control', 'scaling_factor', str(self.scaling_factor)]
        print(command)
        try:
            subprocess.run(command, check=True)
            print('Setting scaling factor to', str(self.scaling_factor))
            # Update the displayed value in the QLineEdit
            self.scaling_factor_edit.setText(str(self.scaling_factor))
        except subprocess.CalledProcessError as e:
            print('Error setting scaling factor')
  
            

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
