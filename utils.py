import os
import subprocess
from PyQt5.QtWidgets import QVBoxLayout, QLabel, QComboBox, QCheckBox


def read_from_file(filename):
    listed = []
    with open(filename, "r") as text:
        Line = text.readline()
        while Line != "":
            listed.append(Line.rstrip("\n"))
            Line = text.readline()

    return listed


def get_net_interfaces():
    # get the list of the network interfaces names
    proc = subprocess.Popen(["ls /sys/class/net"], stdout=subprocess.PIPE, shell=True)
    (out, err) = proc.communicate()
    out = out.decode().split("\n")
    # remove last empty element
    out.pop()
    out.insert(0, "default")
    return out


def create_combo_options_layout(label_txt, options, default_index):
    layout = QVBoxLayout()
    label = QLabel()
    label.setText(label_txt)
    layout.addWidget(label)
    # set size
    label.setFixedSize(200, 30)
    combobox = QComboBox()
    combobox.addItems(options)
    combobox.setCurrentIndex(default_index)
    # set size
    combobox.setFixedSize(200, 30)
    layout.addWidget(combobox)
    return combobox,layout


def create_checkbox_options_layout(label_txt):
    layout = QVBoxLayout()
    label = QLabel()
    label.setText(label_txt)
    label.setFixedSize(200, 30)
    layout.addWidget(label)

    # add checkbox
    checkbox = QCheckBox()
    checkbox.setFixedSize(200, 30)
    layout.addWidget(checkbox)
    checkbox.setStyleSheet(
        "QCheckBox::indicator" "{" "width :40px;" "height : 40px;" "}"
    )

    return checkbox, layout
