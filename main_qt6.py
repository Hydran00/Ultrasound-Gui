import sys
from PyQt6.QtWidgets import QApplication, QWidget, QPushButton, QGridLayout, QTextEdit, QLabel, QSizePolicy, QLineEdit
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont, QColor, QImage, QPixmap
from PyQt6 import QtCore
import subprocess
import utils
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import SetBool
from std_msgs.msg import Bool
from rcl_interfaces.srv import SetParameters
import qdarktheme
import shlex
import cv2
  # Import the qdarkstyle library

class SubprocessButton(QPushButton):
    def __init__(self, command, label, output_widget, parent=None):
        super().__init__(parent)
        self.label = label
        self.output_widget = output_widget
        self.setFixedSize(500, 200)
        self.setText("Start " + self.label)
        self.setFont(QFont('Arial', 12)) 
        self.command = command
        self.process = subprocess.Popen(self.command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.running = False
        self.color_index = 0
        # extract color from qdark theme
        self.inactive_process_button_style = self.styleSheet() 
        self.active_process_button_color = QColor(87,150,244)
        self.set_inactive_process_button_color()
        self.clicked.connect(self.toggle_subprocess)

    def toggle_subprocess(self):
        if self.running:
            self.kill_subprocess()
            if self.label == "Motion Handle":
                subprocess.run(['ros2', 'run', 'controller_manager', 'spawner', 'motion_control_handle', '-c', '/controller_manager'])
        else:
            self.start_subprocess()

    def set_active_process_button_color(self):
        self.setStyleSheet("background-color: " + self.active_process_button_color.name() + ";")

    def set_inactive_process_button_color(self):
        self.setStyleSheet(self.inactive_process_button_style)

    def start_subprocess(self):
        if "haptic" in self.command:
            # disable the other buttons that have "handle" in command
            for button in self.parent().button_textbox_map.keys():
                if button != self and "handle" in button.command:
                    button.setDisabled(True)
        if "handle" in self.command:
            # disable the other buttons that have "handle" in command
            for button in self.parent().button_textbox_map.keys():
                if button != self and "haptic" in button.command:
                    button.setDisabled(True)

        if not self.running:
            self.running = True
            self.set_active_process_button_color()
            self.setText(self.label + " running")           
            self.process = subprocess.Popen(self.command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.read_output()

    def kill_subprocess(self):
        if self.running:
            self.setText(self.label + " terminating")
            self.process.terminate()

    def read_output(self):
        output, error = self.process.communicate()
        output_text = output.decode() + error.decode()
        if self.output_widget is not None:
            self.output_widget.append(output_text)

class MainWindow(QWidget):

    def __init__(self):
        super().__init__()
        self.bounding_box = int(0)
        self.scaling_factor = float(1)

        self.setWindowTitle("Control Panel")
        layout = QGridLayout()
        self.setLayout(layout)
        self.setWindowFlags(QtCore.Qt.WindowType.WindowStaysOnTopHint)

        # Apply the PyQTDark theme
        qdarktheme.setup_theme()

        # Create QLabel to display the current frame
        self.frame_label = QLabel()
        self.frame_label.setSizePolicy(QSizePolicy.Policy.MinimumExpanding, QSizePolicy.Policy.MinimumExpanding)
        layout.addWidget(self.frame_label, 0, 2, -1, 1)  # Span all rows

        # Start the QTimer to update the frame
        self.timer = QTimer(self)
        self.timer.start(40)  # Update every 40 milliseconds (25 frames per second)

        # Dictionary to hold buttons, text boxes, and scroll buttons
        self.button_textbox_map = {}

        # add buttons, text boxes, and scroll buttons
        commands = utils.read_from_file("assets/commands.txt")
        labels = utils.read_from_file("assets/labels.txt")
        for i, command in enumerate(commands):
            output_textbox = QTextEdit()
            output_textbox.setMinimumSize(800, 200)  # Set minimum size
            layout.addWidget(output_textbox, i, 1)

            scroll_button = QPushButton("AutoScroll")
            scroll_button.setFixedSize(200, 30)
            scroll_button.clicked.connect(lambda checked, tb=output_textbox, sb=scroll_button: self.toggle_autoscroll(sb, tb))
            layout.addWidget(scroll_button, i, 2)

            print("Adding button for command: ", command)
            
            button_launch = SubprocessButton(shlex.split(command), "Start " + labels[i], output_textbox)
            layout.addWidget(button_launch, i, 0)
            self.button_textbox_map[button_launch] = (output_textbox, scroll_button)

        # add interactive marker
        cmd = ['ros2', 'run', 'controller_manager', 'spawner', 'motion_control_handle', '-c', '/controller_manager']
        button_launch = SubprocessButton(cmd, "Start Motion Handle", None)
        layout.addWidget(button_launch, len(commands), 0)

        # add interactive marker 
        cmd = ['ros2', 'run', 'controller_manager', 'unspawner', 'motion_control_handle', '-c', '/controller_manager']
        button_launch = SubprocessButton(cmd, "Kill Motion Handle", None)
        layout.addWidget(button_launch, len(commands), 1)

        # Button to toggle bounding_box parameter
        bounding_box_button = QPushButton("Toggle Bounding Box")
        bounding_box_button.setFixedSize(500, 40)
        bounding_box_button.clicked.connect(lambda checked, ros_node=self: ros_node.toggle_bounding_box())
        layout.addWidget(bounding_box_button, len(commands) + 1, 0)

        # QLineEdit to display and edit the value of self.bounding_box
        self.bounding_box_edit = QLineEdit(str(self.bounding_box))
        self.bounding_box_edit.setFixedWidth(100)  # Set the width according to your preference
        self.bounding_box_edit.setReadOnly(True)   # Make it read-only
        layout.addWidget(self.bounding_box_edit, len(commands) + 1, 1)

        # Button to decrease scaling factor
        decrease_scaling_factor_button = QPushButton("Decrease Scaling Factor")
        decrease_scaling_factor_button.setFixedSize(500, 40)
        decrease_scaling_factor_button.clicked.connect(lambda checked, ros_node=self: ros_node.decrease_scaling_factor())
        layout.addWidget(decrease_scaling_factor_button, len(commands) + 3, 0)

        # Button to increase scaling factor
        increase_scaling_factor_button = QPushButton("Increase Scaling Factor")
        increase_scaling_factor_button.setFixedSize(500, 40)
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
        

    def toggle_autoscroll(self, button, text_box):
        is_read_only = text_box.isReadOnly()
        text_box.setReadOnly(not is_read_only)
        if is_read_only:
            button.setStyleSheet("background-color: none; color: black;")
        else:
            button.setStyleSheet("background-color: grey; color: black;")
    def update_frame(self):
        if(not os.path.exists("ultrasound_screen.jpg")):
            return
        frame = cv2.imread('ultrasound_screen.jpg')
        if frame is not None:
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            pixmap = QPixmap.fromImage(QImage(rgb_frame.data, rgb_frame.shape[1], rgb_frame.shape[0], QImage.Format_RGB888))
            self.frame_label.setPixmap(pixmap.scaled(self.frame_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

if __name__ == "__main__":
    rclpy.init(args=None)
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    try:
        sys.exit(app.exec())
    finally:
        # Clean up when the application is closed
        rclpy.shutdown()