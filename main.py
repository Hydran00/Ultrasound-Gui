from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QGridLayout, QTextEdit, QLabel, QSizePolicy
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QPixmap
from PyQt5.QtCore import QProcess, pyqtSlot, QProcess
from PyQt5.QtGui import QImage
import sys, os
import cv2
import subprocess
os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH")
#source ros environment
subprocess.run("assets/source ros_source", shell=True)


import utils

class SubprocessButton(QPushButton):
    def __init__(self, command, label, output_widget, parent=None):
        super().__init__(parent)
        self.label = label
        self.output_widget = output_widget
        self.setFixedSize(300, 150)
        self.setText("Start " + self.label)
        self.setFont(QFont('Arial', 12)) 
        self.command = command
        self.process = QProcess()
        self.running = False
        self.setStyleSheet("background-color: red;")
        self.clicked.connect(self.toggle_subprocess)

    def toggle_subprocess(self):
        if self.running:
            self.kill_subprocess()
        else:
            self.start_subprocess()

    def start_subprocess(self):
        if not self.running:
            self.running = True
            self.setStyleSheet("background-color: green;")
            self.setText(self.label + " running")           
            self.process.start(self.command)
            self.process.readyReadStandardOutput.connect(self.read_output)
            self.process.readyReadStandardError.connect(self.read_output)
            self.process.finished.connect(self.on_finished)
            
    def kill_subprocess(self):
        if self.running:
            self.setText(self.label + " terminating")
            killer = QProcess()
            killer.start("kill -SIGINT " + str(self.process.processId()))
            killed = killer.waitForFinished(1000)

    def read_output(self):
        output = self.process.readAllStandardOutput().data().decode()
        error = self.process.readAllStandardError().data().decode()
        output_text = output + error
        self.output_widget.append(output_text)

    def on_finished(self, exitCode, exitStatus):
        self.running = False
        self.setStyleSheet("background-color: red;")
        self.setText("Start " + self.label)

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Control Panel")
        layout = QGridLayout()
        self.setLayout(layout)
        self.setWindowFlags(Qt.WindowStaysOnTopHint)

        # Create QLabel to display the current frame
        self.frame_label = QLabel()
        # self.frame_label.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)
        self.frame_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(self.frame_label, 0, 2, -1, 1)  # Span all rows

        # Start the QTimer to update the frame
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(40)  # Update every 40 milliseconds (25 frames per second)

        # Dictionary to hold buttons and their associated text boxes
        self.button_textbox_map = {}

        # add buttons and text boxes
        commands = utils.read_from_file("assets/commands.txt")
        labels = utils.read_from_file("assets/labels.txt")
        for i, command in enumerate(commands):
            output_textbox = QTextEdit()
            output_textbox.setMinimumSize(400, 200)  # Set minimum size
            layout.addWidget(output_textbox, i, 1)
            
            button_launch = SubprocessButton(command, labels[i], output_textbox)
            layout.addWidget(button_launch, i, 0)
            self.button_textbox_map[button_launch] = output_textbox

    def update_frame(self):
        if(not os.path.exists("ultrasound_screen.jpg")):
            return
        frame = cv2.imread('ultrasound_screen.jpg')
        if frame is not None:
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            pixmap = QPixmap.fromImage(QImage(rgb_frame.data, rgb_frame.shape[1], rgb_frame.shape[0], QImage.Format_RGB888))
            self.frame_label.setPixmap(pixmap.scaled(self.frame_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
