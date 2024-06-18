from PyQt5.QtWidgets import QPushButton
from PyQt5.QtGui import QFont, QColor
from PyQt5.QtCore import QProcess

class BaseButton(QPushButton):
    def __init__(self, command, label, output_widget, parent=None):
        super().__init__(parent)
        self.label = label
        self.output_widget = output_widget
        # self.setFixedSize(500, 200)
        self.setText(self.label)
        self.setFont(QFont('Arial', 12)) 
        self.command = command
        self.process = QProcess()
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
                os.system('ros2 run controller_manager spawner motion_control_handle -c /controller_manager')
        else:
            self.start_subprocess()

    def set_active_process_button_color(self):
        self.setStyleSheet("background-color: " + self.active_process_button_color.name() + ";")

    def set_inactive_process_button_color(self):
        self.setStyleSheet(self.inactive_process_button_style)

    def start_subprocess(self):
        if not self.running:
            self.running = True
            # self.setStyleSheet("background-color: green;")
            self.set_active_process_button_color()
            self.setText(self.label + " running")           
            self.process.start(self.command)
            self.process.readyReadStandardOutput.connect(self.read_output)
            self.process.readyReadStandardError.connect(self.read_output)
            self.process.finished.connect(self.on_finished)
            
    def kill_subprocess(self):
        # VR app must be closed manually to avoid crashing SteamVR
        if self.running and "VR" not in self.label: 
            self.setText(self.label + " terminating")
            killer = QProcess()
            killer.start("kill -SIGINT " + str(self.process.processId()))
            killed = killer.waitForFinished(1000)

    def read_output(self):
        output = self.process.readAllStandardOutput().data().decode()[:-1]
        error = self.process.readAllStandardError().data().decode()[:-1]
        output_text = output + error
        if self.output_widget is not None:
            old_cursor = self.output_widget.textCursor()
            old_value = self.output_widget.verticalScrollBar().value()
            scrolled_down = old_value == self.output_widget.verticalScrollBar().maximum()
            self.output_widget.moveCursor(old_cursor.End)
            self.output_widget.append(output_text)
            if(old_cursor.hasSelection() or not scrolled_down):
                self.output_widget.setTextCursor(old_cursor)
                self.output_widget.verticalScrollBar().setValue(old_value)
            else:
                if scrolled_down:
                    self.output_widget.moveCursor(old_cursor.End)
                    self.output_widget.verticalScrollBar().setValue(self.output_widget.verticalScrollBar().maximum())

    def on_finished(self, exitCode, exitStatus):
        self.running = False
        # self.setStyleSheet("background-color: red;")
        self.set_inactive_process_button_color()
        self.setText(self.label)