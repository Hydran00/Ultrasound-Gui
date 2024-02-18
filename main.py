import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QGridLayout
from PyQt5.QtCore import QProcess, pyqtSlot
# font
from PyQt5.QtGui import QFont

class SubprocessButton(QPushButton):
    def __init__(self, command, title, parent=None):
        super().__init__(parent)
        self.title = title
        self.setFixedSize(300, 50)
        self.setText("Start " + self.title)
        self.setFont(QFont('Arial', 15)) 
        self.command = command
        self.process = QProcess()
        self.running = False
        self.setStyleSheet("background-color: red;")
        self.clicked.connect(self.toggle_subprocess)

    @pyqtSlot()
    def toggle_subprocess(self):
        if self.running:
            self.kill_subprocess()
        else:
            self.start_subprocess()

    def start_subprocess(self):
        if not self.running:
            self.running = True
            self.setStyleSheet("background-color: green;")
            self.setText(self.title + " running")
            self.process.start(self.command[0], self.command[1:])
            print(f"Started {self.command}")
            self.process.finished.connect(self.on_finished)

    def kill_subprocess(self):
        if self.running:
            print(f"Killing {self.title}")
            self.process.kill()
            self.on_finished(exitCode=None, exitStatus=0)

    @pyqtSlot(int, QProcess.ExitStatus)
    def on_finished(self, exitCode, exitStatus):
        self.running = False
        self.setStyleSheet("background-color: red;")
        self.setText("Start " + self.title)


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Subprocess Controller")
        layout = QGridLayout()
        self.setLayout(layout)

        commands = [
            ["ros2 launch ur_robot_driver ur_robot_driver.launch.py"],
            ["./home/hydran00/ecography_ws/tcp_endpoint.sh"],
            ["echo", "Process 2"],
            ["echo", "Process 3"],
            ["echo", "Process 4"]
        ]

        title = "Robot environment"
        button_launch = SubprocessButton(commands[0], title)
        layout.addWidget(button_launch, 0, 0)
        
        title = "ROS-TCP Connector"
        button_launch = SubprocessButton(commands[1], title)
        layout.addWidget(button_launch, 0, 1)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
