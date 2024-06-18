from .base import BaseButton
import utils
class TcpEndpointButton(BaseButton):
    def __init__(self, command, label, output_widget, net_interface_switch, parent=None):
        super().__init__(command, label, output_widget, parent)
        self.net_interface_switch = net_interface_switch
        # self.command = command + " interface:="+utils.get_net_interfaces()
    # re-implement the start_subprocess method adding the possibility to add the ethernet interface
    def start_subprocess(self):
        if not self.running:
            self.running = True
            self.set_active_process_button_color()
            self.setText(self.label + " running")           
            self.process.start(self.command + " interface:="+self.net_interface_switch.currentItem().text())
            self.process.readyReadStandardOutput.connect(self.read_output)
            self.process.readyReadStandardError.connect(self.read_output)
            self.process.finished.connect(self.on_finished)