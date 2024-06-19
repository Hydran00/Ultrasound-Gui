from .base import BaseButton
import utils
from PyQt5.QtCore import QProcess
from PyQt5.QtCore import QTimer


class HandleButton(BaseButton):
    def __init__(self, parent=None):
        super().__init__(None, "Motion Control\n Handle", None, parent)
        self.start_command = 'ros2 run controller_manager spawner motion_control_handle -c /controller_manager'
        self.stop_command = 'ros2 run controller_manager unspawner motion_control_handle -c /controller_manager'
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.force_stop)
        
    # override the toggle_subprocess method
    def toggle_subprocess(self):
        print("toggle_subprocess custom")
        if self.running:
            self.stop_handle()
        else:
            self.set_active_process_button_color()
            self.start_handle()

    def start_handle(self):
        self.running = True
        self.setText(self.label + " running")           
        self.process.start(self.start_command)
        self.setEnabled(False)
        self.process.finished.connect(self.on_finished)
        self.timer.start(3000)

    def stop_handle(self):
        self.set_inactive_process_button_color()
        self.process.start(self.stop_command)
        self.running = False
        self.setEnabled(False)
        self.setText("Motion\n Handle")
        self.process.finished.connect(self.on_finished)
        self.timer.start(3000)
    
    def on_finished(self, exitCode, exitStatus):
        self.setEnabled(True)
        self.timer.stop()

    def force_stop(self):
        self.kill_subprocess()
        self.timer.stop()
        self.setEnabled(True)
        current_text = self.text()
        self.setText(current_text + "\nfailed")
        self.running = False
        self.set_inactive_process_button_color()
        self.setEnabled(True)
        