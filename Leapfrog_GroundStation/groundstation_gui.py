import sys
from PyQt5 import QtWidgets
import pyqtgraph as pg
import serial

class GroundStationGUI(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()
        # Placeholder for serial connection
        self.serial_port = None

    def init_ui(self):
        layout = QtWidgets.QVBoxLayout()

        # Live plot area
        self.plot_widget = pg.PlotWidget(title="Live Telemetry")
        layout.addWidget(self.plot_widget)

        # Button to send command
        self.command_button = QtWidgets.QPushButton("Send ACS Start Command")
        self.command_button.clicked.connect(self.send_command)
        layout.addWidget(self.command_button)

        # Text area for telemetry/debug output
        self.text_area = QtWidgets.QTextEdit()
        self.text_area.setReadOnly(True)
        layout.addWidget(self.text_area)

        self.setLayout(layout)
        self.setWindowTitle("Leapfrog Ground Station GUI (Starter)")
        self.resize(600, 400)

    def send_command(self):
        # Placeholder: send a command over serial
        self.text_area.append("[DEBUG] Command sent: ACSsystem Enable")
        # if self.serial_port:
        #     self.serial_port.write(b"ACSsystem Enable\n")

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    gui = GroundStationGUI()
    gui.show()
    sys.exit(app.exec_()) 