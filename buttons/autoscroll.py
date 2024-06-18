from PyQt5.QtWidgets import QCheckBox
from PyQt5.QtGui import QFont, QColor
from PyQt5.QtCore import QProcess

class AutoScrollButton(QCheckBox):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setText("Autoscroll")
        self.current_cursor_position = 0
    
    def toggle_autoscroll(self, button, text_box):        
        # text_box.ensureCursorVisible()
        if button.isChecked():
            text_output.moveCursor(old_cursor.End)