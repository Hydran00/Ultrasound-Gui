import subprocess
import sys, os
import argparse

from PyQt5.QtCore import QLibraryInfo
import cv2
os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = QLibraryInfo.location(
    QLibraryInfo.PluginsPath
)

# from PyQt5 import QtCore
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QPushButton,
    QGridLayout,
    QPlainTextEdit,
    QLabel,
    # QSizePolicy,
    # QLineEdit,
    QListWidget,
    QVBoxLayout,
    QHBoxLayout,
    # QCheckBox,
)
from PyQt5.QtGui import QDoubleValidator
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QPixmap
from PyQt5.QtCore import QProcess, pyqtSlot
from PyQt5.QtGui import QImage, QColor
from PyQt5 import QtGui
import utils
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# from std_msgs.msg import Bool
import qdarkstyle  # Import the qdarkstyle library

from buttons.base import BaseButton
from buttons.motion_handle import HandleButton
from buttons.tcp_endpoint import TcpEndpointButton
from buttons.ft_calibration import FTCalibrationButton
# from us_stream import USImageSubscriber

# { 'param name' , ['default idx or value', ['options']] }
ARGS = {
    "use_ft_sensor": False,  # True or False
    "camera_type": [1, ["realsense", "zed"]],  # realsense or zed
    "robot_type": [0, ["kuka", "ur3e"]],  # kuka or ur3e
    "video_stream": False,  # True or False
    "point_cloud_stream": True,  # True or False
    "encode_streams": False,  # True or False
    "use_fixtures": False,  # True or False
    "delay": 0.0,
}


class MainWindow(QWidget):

    def __init__(self, config_file=None):
        super().__init__()
        # if config_file is not None:
        #     with open(config_file, "r") as f:
        #         for line in f:
        #             key, value = line.split("=")
        #             if key in ARGS:
        #                 if key == "delay":
        #                     ARGS[key] = float(value)
        #                 elif key == "use_ft_sensor":
        #                     ARGS[key] = value == "True"
        #                 else:
        #                     ARGS[key] = value

        self.bounding_box = int(0)
        self.scaling_factor = float(1)

        self.setWindowTitle("Control Panel")
        self.setWindowIcon(QtGui.QIcon("assets/logo.png"))
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
        switch_camera_type, camera_selection_layout = utils.create_combo_option_layout(
            "Camera\ntype:", ARGS["camera_type"][1], ARGS["camera_type"][0]
        )
        switch_robot_type, robot_selection_layout = utils.create_combo_option_layout(
            "Robot\ntype:", ARGS["robot_type"][1], ARGS["robot_type"][0]
        )
        checkbox_ft_sensor, use_ft_sensor_layout = utils.create_checkbox_option_layout(
            "Use FT\nSensor", ARGS["use_ft_sensor"]
        )
        checkbox_video_stream, video_stream_layout = (
            utils.create_checkbox_option_layout("Video\nStream", ARGS["video_stream"])
        )
        checkbox_point_cloud, point_cloud_stream_layout = (
            utils.create_checkbox_option_layout(
                "Point Cloud\nStream", ARGS["point_cloud_stream"]
            )
        )
        checkbox_encode_stream, encode_streams_layout = (
            utils.create_checkbox_option_layout(
                "Encode\nStreams", ARGS["encode_streams"]
            )
        )
        use_fixtures, use_fixtures_layout = utils.create_checkbox_option_layout(
            "Use\nfixtures", ARGS["use_fixtures"]
        )
        delay, delay_layout = utils.create_double_input_layout("Delay", ARGS["delay"])

        args_keys = list(ARGS.keys())
        options_button_follower = {
            "checkboxes": [
                (args_keys[0], checkbox_ft_sensor),
                (args_keys[3], checkbox_video_stream),
                (args_keys[4], checkbox_point_cloud),
                (args_keys[5], checkbox_encode_stream),
            ],
            "switches": [
                (args_keys[1], switch_camera_type),
                (args_keys[2], switch_robot_type),
            ],
            "values": [],
        }
        options_button_leader = {
            "checkboxes": [
                (args_keys[0], checkbox_ft_sensor),
                (args_keys[3], checkbox_video_stream),
                (args_keys[4], checkbox_point_cloud),
                (args_keys[6], use_fixtures),
            ],
            "switches": [
                (args_keys[2], switch_robot_type),
            ],
            "values": [
                (args_keys[7], delay),
            ],
        }

        # add to layout
        opt_layout.addLayout(use_ft_sensor_layout, Qt.AlignCenter)
        opt_layout.addLayout(camera_selection_layout, Qt.AlignCenter)
        opt_layout.addLayout(robot_selection_layout, Qt.AlignCenter)
        opt_layout.addLayout(video_stream_layout, Qt.AlignCenter)
        opt_layout.addLayout(point_cloud_stream_layout, Qt.AlignCenter)
        opt_layout.addLayout(encode_streams_layout, Qt.AlignCenter)
        opt_layout.addLayout(use_fixtures_layout, Qt.AlignCenter)
        opt_layout.addLayout(delay_layout, Qt.AlignCenter)
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
                button_launch = TcpEndpointButton(
                    command, labels[i], output_textbox, self.net_interface_switch
                )
                net_selection_layout.addWidget(self.net_interface_switch)
                text_box_command_layout.addLayout(net_selection_layout, Qt.AlignCenter)

            else:
                if "follower" in command:
                    button_launch = BaseButton(
                        command,
                        labels[i],
                        output_textbox,
                        None,
                        options_button_follower,
                    )
                elif "leader" in command:
                    button_launch = BaseButton(
                        command, labels[i], output_textbox, None, options_button_leader
                    )
                else:
                    button_launch = BaseButton(command, labels[i], output_textbox)

            button_launch.setMinimumSize(150, 100)
            layout.addWidget(button_launch, i, 0)
            self.button_textbox_map[button_launch] = output_textbox
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

        # save options button
        button_save_options = QPushButton("Save\nOptions")
        button_save_options.setText("Save\nOptions")
        button_save_options.setFont(QFont("Arial", 12))

        def save_options():
            # Save the options to a file
            with open("assets/options.txt", "w") as f:
                for key, value in ARGS.items():
                    f.write(f"{key}={value}\n")

        button_save_options.clicked.connect(save_options)
        handle_layout.addWidget(button_save_options, Qt.AlignCenter)

        layout.addLayout(handle_layout, len(commands), 0)
        # ultrasound image feedback
        # self.us_image_feedback = USImageSubscriber(layout)
        app_layout.addLayout(layout)

    def resizeEvent(self, event):
        # Resize the pixmap when the window is resized
        if hasattr(self, "us_image_feedback"):
            us_image_label = self.us_image_feedback.us_image_label
            if us_image_label.pixmap() is not None:
                scaled_pixmap = us_image_label.pixmap().scaled(
                    us_image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
                )
                us_image_label.setPixmap(scaled_pixmap)
        super().resizeEvent(event)


if __name__ == "__main__":
    # get arguments
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--load_conf", type=str, default="assets/options.txt", required=False
    )

    # rclpy.init(args=None)
    app = QApplication(sys.argv)
    window = MainWindow(parser.parse_args().load_conf)
    window.show()
    try:
        sys.exit(app.exec_())
    finally:
        # Clean up when the application is closed
        # rclpy.shutdown()
        pass
