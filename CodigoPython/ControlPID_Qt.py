import sys
import serial
import threading
import time
import numpy as np

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QLineEdit
)
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QFont

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


# =============================
# CONFIGURACIÓN SERIAL
# =============================
PORT = "COM3"   # Cambiar si es necesario
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)


# =============================
# CLASE PRINCIPAL
# =============================
class PIDDashboard(QWidget):

    def __init__(self):
        super().__init__()

        self.setWindowTitle("Control PID - Motores ESP32")
        self.setGeometry(200, 200, 900, 600)

        self.rpm1 = 0
        self.rpm2 = 0

        self.data1 = []
        self.data2 = []
        self.time_data = []

        self.init_ui()
        self.start_serial_thread()

        # Timer para actualizar gráfica
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(200)

    # =============================
    # INTERFAZ
    # =============================
    def init_ui(self):
        layout = QVBoxLayout()
        font_big = QFont("Arial", 14)

        self.label_rpm1 = QLabel("Motor 1 RPM: 0")
        self.label_rpm1.setFont(font_big)

        self.label_rpm2 = QLabel("Motor 2 RPM: 0")
        self.label_rpm2.setFont(font_big)

        layout.addWidget(self.label_rpm1)
        layout.addWidget(self.label_rpm2)

        # =========================
        # MOTOR 1
        # =========================
        motor1_layout = QHBoxLayout()

        self.sp1_input = QLineEdit()
        self.sp1_input.setPlaceholderText("Setpoint M1")

        self.kp1_input = QLineEdit()
        self.kp1_input.setPlaceholderText("Kp1")

        self.ki1_input = QLineEdit()
        self.ki1_input.setPlaceholderText("Ki1")

        self.kd1_input = QLineEdit()
        self.kd1_input.setPlaceholderText("Kd1")

        motor1_layout.addWidget(self.sp1_input)
        motor1_layout.addWidget(self.kp1_input)
        motor1_layout.addWidget(self.ki1_input)
        motor1_layout.addWidget(self.kd1_input)

        layout.addLayout(motor1_layout)

        # =========================
        # MOTOR 2
        # =========================
        motor2_layout = QHBoxLayout()

        self.sp2_input = QLineEdit()
        self.sp2_input.setPlaceholderText("Setpoint M2")

        self.kp2_input = QLineEdit()
        self.kp2_input.setPlaceholderText("Kp2")

        self.ki2_input = QLineEdit()
        self.ki2_input.setPlaceholderText("Ki2")

        self.kd2_input = QLineEdit()
        self.kd2_input.setPlaceholderText("Kd2")

        motor2_layout.addWidget(self.sp2_input)
        motor2_layout.addWidget(self.kp2_input)
        motor2_layout.addWidget(self.ki2_input)
        motor2_layout.addWidget(self.kd2_input)

        layout.addLayout(motor2_layout)

        self.btn_send = QPushButton("Enviar Parámetros")
        self.btn_send.clicked.connect(self.send_parameters)
        layout.addWidget(self.btn_send)

        # Gráfica
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)

        layout.addWidget(self.canvas)

        self.setLayout(layout)

    # =============================
    # SERIAL THREAD
    # =============================
    def start_serial_thread(self):
        thread = threading.Thread(target=self.read_serial)
        thread.daemon = True
        thread.start()

    def read_serial(self):
        while True:
            try:
                line = ser.readline().decode().strip()

                if line.startswith("DATA"):
                    parts = line.split(",")

                    self.rpm1 = float(parts[1])
                    self.rpm2 = float(parts[2])

                    self.data1.append(self.rpm1)
                    self.data2.append(self.rpm2)
                    self.time_data.append(time.time())

                elif line != "":
                    print("ESP32:", line)

            except Exception as e:
                print("Error lectura:", e)

    # =============================
    # ACTUALIZAR GRÁFICA
    # =============================
    def update_plot(self):
        self.label_rpm1.setText(f"Motor 1 RPM: {self.rpm1:.2f}")
        self.label_rpm2.setText(f"Motor 2 RPM: {self.rpm2:.2f}")

        self.ax.clear()
        self.ax.plot(self.data1[-100:], label="Motor 1")
        self.ax.plot(self.data2[-100:], label="Motor 2")
        self.ax.legend()
        self.ax.set_ylabel("RPM")
        self.ax.set_xlabel("Tiempo")
        self.canvas.draw()

    # =============================
    # ENVIAR PARÁMETROS
    # =============================
    def send_parameters(self):

        try:
            # ===== MOTOR 1 =====
            if self.sp1_input.text():
                ser.write(f"m1={self.sp1_input.text()}\n".encode())

            if self.kp1_input.text():
                ser.write(f"kp1={self.kp1_input.text()}\n".encode())

            if self.ki1_input.text():
                ser.write(f"ki1={self.ki1_input.text()}\n".encode())

            if self.kd1_input.text():
                ser.write(f"kd1={self.kd1_input.text()}\n".encode())

            # ===== MOTOR 2 =====
            if self.sp2_input.text():
                ser.write(f"m2={self.sp2_input.text()}\n".encode())

            if self.kp2_input.text():
                ser.write(f"kp2={self.kp2_input.text()}\n".encode())

            if self.ki2_input.text():
                ser.write(f"ki2={self.ki2_input.text()}\n".encode())

            if self.kd2_input.text():
                ser.write(f"kd2={self.kd2_input.text()}\n".encode())

            print("Parámetros enviados correctamente")

        except Exception as e:
            print("Error enviando:", e)







# =============================
# MAIN
# =============================
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = PIDDashboard()
    window.show()
    sys.exit(app.exec_())