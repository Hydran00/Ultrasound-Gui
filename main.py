import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QGridLayout, QTextEdit
from PyQt5.QtCore import Qt, QProcess, pyqtSlot
from PyQt5.QtGui import QFont
import utils

class SubprocessButton(QPushButton):
    def __init__(self, command, label, output_widget, parent=None):
        super().__init__(parent)
        self.label = label
        self.output_widget = output_widget
        self.setFixedSize(500, 200)
        self.setText("Start " + self.label)
        self.setFont(QFont('Arial', 18)) 
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
            self.setText(self.label + " running")           
            self.process.start(self.command)
            print(self.command)
            self.process.readyReadStandardOutput.connect(self.read_output)
            self.process.readyReadStandardError.connect(self.read_output)
            self.process.finished.connect(self.on_finished)
            
    def kill_subprocess(self):
        if self.running:
            self.setText("Terminating")
            killer = QProcess()
            print("Killing PID", self.process.processId())
            # os.kill(self.process.processId(), signal.SIGINT)
            killer.start("kill -SIGINT " + str(self.process.processId()))
            killed = killer.waitForFinished(1000)
    @pyqtSlot()
    def read_output(self):
        output = self.process.readAllStandardOutput().data().decode()
        error = self.process.readAllStandardError().data().decode()
        output_text = output + error
        self.output_widget.append(output_text)

    @pyqtSlot(int, QProcess.ExitStatus)
    def on_finished(self, exitCode, exitStatus):
        print(f"{self.label} finished with code {exitCode}")
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
        
        # Dictionary to hold buttons and their associated text boxes
        self.button_textbox_map = {}

        # add buttons
        commands = utils.read_from_file("commands.txt")
        labels = utils.read_from_file("labels.txt")
        for i, command in enumerate(commands):
            output_textbox = QTextEdit()
            output_textbox.setMinimumWidth(500)
            layout.addWidget(output_textbox, i, 1)
            button_launch = SubprocessButton(command, labels[i], output_textbox)
            layout.addWidget(button_launch, i, 0)
            self.button_textbox_map[button_launch] = output_textbox

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
