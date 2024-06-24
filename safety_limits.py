    #     # Button to toggle bounding_box parameter
    #     bounding_box_button = QPushButton("Toggle Bounding Box")
    #     bounding_box_button.setMinimumSize(150, 40)
    #     bounding_box_button.clicked.connect(lambda checked, ros_node=self: ros_node.toggle_bounding_box())
    #     layout.addWidget(bounding_box_button, len(commands) + 1, 0)

    #     # QLineEdit to display and edit the value of self.bounding_box
    #     self.bounding_box_edit = QLineEdit(str(self.bounding_box))

    #     self.bounding_box_edit.setFixedWidth(100)  # Set the width according to your preference
    #     self.bounding_box_edit.setReadOnly(True)   # Make it read-only
    #     layout.addWidget(self.bounding_box_edit, len(commands) + 1, 1)

    #     # Button to decrease scaling factor
    #     decrease_scaling_factor_button = QPushButton("Decrease Scaling Factor")
    #     decrease_scaling_factor_button.setMinimumSize(150, 40)
    #     decrease_scaling_factor_button.clicked.connect(lambda checked, ros_node=self: ros_node.decrease_scaling_factor())
    #     layout.addWidget(decrease_scaling_factor_button, len(commands) + 3, 0)

    #     # Button to increase scaling factor
    #     increase_scaling_factor_button = QPushButton("Increase Scaling Factor")
    #     increase_scaling_factor_button.setMinimumSize(150, 40)
    #     increase_scaling_factor_button.clicked.connect(lambda checked, ros_node=self: ros_node.increase_scaling_factor())
    #     layout.addWidget(increase_scaling_factor_button, len(commands) + 4, 0)

    #     # QLineEdit to display the value of self.scaling_factor
    #     self.scaling_factor_edit = QLineEdit(str(self.scaling_factor))
    #     self.scaling_factor_edit.setFixedWidth(100)  # Set the width according to your preference
    #     self.scaling_factor_edit.setReadOnly(True)   # Make it read-only
    #     layout.addWidget(self.scaling_factor_edit, len(commands) + 4, 1)

    #     # ultrasound image feedback
    #     us_image_feedback = USImageSubscriber(layout)
        





    # def toggle_bounding_box(self):
    #     self.bounding_box = int(not self.bounding_box)
    #     command = ['ros2', 'param', 'set', 'haptic_control', 'bounding_box', str(self.bounding_box)]
    #     print(command)
    #     try:
    #         subprocess.run(command, check=True)
    #         print('Toggled bounding box to', bool(self.bounding_box))
    #         # Update the displayed value in the QLineEdit
    #         self.bounding_box_edit.setText(str(self.bounding_box))
    #     except subprocess.CalledProcessError as e:
    #         print('Error setting bounding box')

    # def decrease_scaling_factor(self):
    #     self.scaling_factor = self.scaling_factor - 0.25 

    #     if self.scaling_factor < 0.0:
    #         self.scaling_factor = 0.0

    #     command = ['ros2', 'param', 'set', 'haptic_control', 'scaling_factor', str(self.scaling_factor)]
    #     print(command)
    #     try:
    #         subprocess.run(command, check=True)
    #         print('Setting scaling factor to', str(self.scaling_factor))
    #         # Update the displayed value in the QLineEdit
    #         self.scaling_factor_edit.setText(str(self.scaling_factor))
    #     except subprocess.CalledProcessError as e:
    #         print('Error setting scaling factor')

    # def increase_scaling_factor(self):
    #     self.scaling_factor = self.scaling_factor + 0.25 

    #     if self.scaling_factor > 1.0:
    #         self.scaling_factor = 1.0

    #     command = ['ros2', 'param', 'set', 'haptic_control', 'scaling_factor', str(self.scaling_factor)]
    #     print(command)
    #     try:
    #         subprocess.run(command, check=True)
    #         print('Setting scaling factor to', str(self.scaling_factor))
    #         # Update the displayed value in the QLineEdit
    #         self.scaling_factor_edit.setText(str(self.scaling_factor))
    #     except subprocess.CalledProcessError as e:
    #         print('Error setting scaling factor')
