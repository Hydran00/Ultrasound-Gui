from PyQt5.QtWidgets import QTextEdit
class TextOutput(QTextEdit):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setReadOnly(True)
        self.setMinimumSize(400, 200)
        self.autoscroll = False
        self.cursor = self.textCursor()
        self.cursor.movePosition(self.cursor.End)
        self.scrolled_to_bottom = self.cursor.value() == self.cursor.maximum()
        