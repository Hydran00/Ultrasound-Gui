# this module is used to monitor the status of nodes
# and update the status using visual green/red circles
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QPainter, QColor, QFont

class StatusUpdater()
    def __init__(self, cmd):
        self.running = False
        self.command = cmd
        # Simulate receiving status updates
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_node_status)
        self.timer.start(1000)
        self.node_name = "test"
        def update_node_status(self):
            # checks if a process is running
            
        self.node_status_widget.update_status(status)
        

class NodeStatusWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(100, 100)
        self.status = {}

    def update_status(self, status):
        self.status = status
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        node_count = len(self.status)
        if node_count == 0:  # Handle division by zero
            return

        circle_diameter = min(self.width(), self.height()) / (node_count * 2)
        y = self.height() / 2

        for i, (node, is_running) in enumerate(self.status.items()):
            x = (i + 1) * (circle_diameter * 2)
            color = Qt.green if is_running else Qt.red
            painter.setBrush(QColor(color))
            painter.drawEllipse(int(x - circle_diameter / 2), int(y - circle_diameter / 2), int(circle_diameter), int(circle_diameter))

            painter.setPen(QColor(Qt.black))
            painter.setFont(QFont('Arial', 10))
            painter.drawText(int(x - circle_diameter / 2), int(y - circle_diameter / 2), int(circle_diameter), int(circle_diameter), Qt.AlignCenter, node)

