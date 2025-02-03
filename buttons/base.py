from PyQt5.QtWidgets import QPushButton
from PyQt5.QtGui import QFont, QColor
from PyQt5.QtCore import QProcess


class BaseButton(QPushButton):
    def __init__(
        self, command, label, output_widget, parent=None, options_buttons=None
    ):
        super().__init__(parent)
        self.label = label
        self.output_widget = output_widget
        self.setText(self.label)
        self.setFont(QFont("Arial", 12))
        self.command = command
        self.process = QProcess()
        self.running = False
        self.color_index = 0
        self.options_buttons = options_buttons
        # extract color from qdark theme
        self.inactive_process_button_style = self.styleSheet()
        self.active_process_button_color = QColor(87, 150, 244)

        self.set_inactive_process_button_color()
        self.clicked.connect(self.toggle_subprocess)

    def toggle_subprocess(self):
        if self.running:
            self.kill_subprocess()
        else:
            self.set_active_process_button_color()
            self.start_subprocess()

    def set_active_process_button_color(self):
        self.setStyleSheet(
            "background-color: " + self.active_process_button_color.name() + ";"
        )

    def set_inactive_process_button_color(self):
        self.setStyleSheet(self.inactive_process_button_style)

    def start_subprocess(self):
        self.running = True
        # self.setStyleSheet("background-color: green;")
        self.setText(self.label + " running")

        opt_suffix = ""
        opt_command = self.command
        if self.options_buttons is not None:
            for i in range(len(self.options_buttons["switches"])):
                (opt_name, switch) = self.options_buttons["switches"][i]
                opt_command += " " + opt_name + ":=" + switch.currentText()

            for i in range(len(self.options_buttons["checkboxes"])):
                (opt_name, button) = self.options_buttons["checkboxes"][i]
                if button.isChecked():
                    opt_command += " " + opt_name + ":=true"
                else:
                    opt_command += " " + opt_name + ":=false"
            for i in range(len(self.options_buttons["values"])):
                (opt_name, button) = self.options_buttons["values"][i]
                opt_command += " " + opt_name + ":=" + button.text()

            # print("Command is: ", opt_command)
        # print command to the output widget
        self.output_widget.appendPlainText(opt_command)

        self.process.start(opt_command)
        self.process.readyReadStandardOutput.connect(self.read_output)
        self.process.finished.connect(self.on_finished)

    def kill_subprocess(self):
        # VR app must be closed manually to avoid crashing SteamVR
        if "VR" not in self.label:
            self.setText(self.label + " terminating")
            killer = QProcess()
            killer.start("kill -SIGINT " + str(self.process.processId()))
            killed = killer.waitForFinished(1000)

    # def read_output(self):
    #     if self.output_widget is not None:
    #         output = self.process.readAllStandardOutput().data().decode().strip()
    #         error = self.process.readAllStandardError().data().decode().strip()
    #         output_text = output + error
    #         old_cursor = self.output_widget.textCursor()
    #         old_value = self.output_widget.verticalScrollBar().value()
    #         self.output_widget.moveCursor(old_cursor.End)
    #         scrolled_down = old_value == self.output_widget.verticalScrollBar().maximum()
    #         self.output_widget.insertPlainText(output_text)
    #         if(old_cursor.hasSelection() or not scrolled_down):
    #             self.output_widget.setTextCursor(old_cursor)
    #             self.output_widget.verticalScrollBar().setValue(old_value)
    #         else:
    #             if scrolled_down:
    #                 self.output_widget.moveCursor(old_cursor.End)
    #                 self.output_widget.verticalScrollBar().setValue(self.output_widget.verticalScrollBar().maximum())
    def read_output(self):
        if self.output_widget is not None:
            # Read output and error
            output = self.process.readAllStandardOutput().data().decode().strip()
            error = self.process.readAllStandardError().data().decode().strip()
            output_text = output + error

            # Get current scrollbar position and scroll state
            scrollbar = self.output_widget.verticalScrollBar()
            scrolled_down = scrollbar.value() == scrollbar.maximum()

            # Append text to output widget
            self.output_widget.appendPlainText(output_text)

            # Limit the number of lines to prevent memory issues
            max_lines = 1000  # Adjust based on memory requirements
            if self.output_widget.blockCount() > max_lines:
                cursor = self.output_widget.textCursor()
                cursor.setPosition(0)
                cursor.movePosition(
                    cursor.Down, cursor.KeepAnchor, cursor.blockNumber() - max_lines
                )
                cursor.removeSelectedText()

            # Scroll to bottom only if previously at the bottom
            if scrolled_down:
                scrollbar.setValue(scrollbar.maximum())

    def on_finished(self, exitCode, exitStatus):
        self.running = False
        self.set_inactive_process_button_color()
        self.setText(self.label)
