import sys,os,signal
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QGridLayout
from PyQt5.QtCore import QProcess, pyqtSlot
# font
from PyQt5.QtGui import QFont
import utils
class SubprocessButton(QPushButton):
    def __init__(self, command, label, parent=None):
        super().__init__(parent)
        self.label = label
        self.setFixedSize(300, 50)
        self.setText("Start " + self.label)
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
            self.setText(self.label + " running")           
            self.process.start(self.command)
            print(self.command)
            # print(f"Started {self.label}")
            self.process.finished.connect(self.on_finished)
            
    def kill_subprocess(self):
        if self.running:
            self.setText(self.label + " terminating")
            killer = QProcess()
            print("Killing PID", self.process.processId())
            # os.kill(self.process.processId(), signal.SIGINT)
            killer.start("kill -SIGINT " + str(self.process.processId()))
            killed = killer.waitForFinished(1000)
    @pyqtSlot(int, QProcess.ExitStatus)
    def on_finished(self, exitCode, exitStatus):
        print(f"{self.label} finished with code {exitCode}")
        self.running = False
        self.setStyleSheet("background-color: red;")
        self.setText("Start " + self.label)


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Subprocess Controller")
        layout = QGridLayout()
        self.setLayout(layout)
        
        # read cmds and labels from file
        commands = utils.read_from_file("commands.txt")
        labels = utils.read_from_file("labels.txt")
        if(len(commands) != len(labels)):
            print("Error: commands and labels have different lengths")
            sys.exit(1)
        # add buttons
        print("Commands is ",commands)
        for button in range(len(commands)):
            print("Creating button for")
            print(commands[button])
            print(labels[button])
            button_launch = SubprocessButton(commands[button], labels[button])
            layout.addWidget(button_launch, 0, button)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
