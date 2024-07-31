#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import sys
from threading import Thread
from ar_gripper.interface import ARGripper
from python_qt_binding.QtWidgets import QMainWindow, QPushButton, QApplication
from python_qt_binding.QtCore import QTimer


class GripperGUI(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self._node = node
        self.gripper = ARGripper("ar_gripper/main", self._node)

        self.init_ui()

    def init_ui(self):

        calibrate_button = QPushButton("Calibrate", self)
        calibrate_button.resize(100, 30)
        calibrate_button.clicked.connect(self.gripper.calibrate)
        calibrate_button.move(50, 10)
        calibrate_button.show()

        release_button = QPushButton("Release", self)
        release_button.resize(200, 200)
        release_button.clicked.connect(self.gripper.release)
        release_button.move(50, 50)

        hard_close_button = QPushButton("Hard\nClose", self)
        hard_close_button.resize(100, 200)
        hard_close_button.clicked.connect(self.gripper.hard_close)
        hard_close_button.move(250, 50)

        hard_close_button = QPushButton("Soft\nClose", self)
        hard_close_button.resize(100, 200)
        hard_close_button.clicked.connect(self.gripper.soft_close)
        hard_close_button.move(350, 50)

        open_button = QPushButton("Open", self)
        open_button.clicked.connect(self.gripper.open)
        open_button.resize(200, 200)
        open_button.move(450, 50)

        goto_button = QPushButton("0%", self)
        goto_button.resize(100, 200)
        goto_button.clicked.connect(self.submit_goto1)
        goto_button.move(50, 250)

        goto_button = QPushButton("10%", self)
        goto_button.resize(100, 200)
        goto_button.clicked.connect(self.submit_goto2)
        goto_button.move(150, 250)

        goto_button = QPushButton("20%", self)
        goto_button.resize(100, 200)
        goto_button.clicked.connect(self.submit_goto3)
        goto_button.move(250, 250)

        goto_button = QPushButton("30%", self)
        goto_button.resize(100, 200)
        goto_button.clicked.connect(self.submit_goto4)
        goto_button.move(350, 250)

        goto_button = QPushButton("40%", self)
        goto_button.resize(100, 200)
        goto_button.clicked.connect(self.submit_goto5)
        goto_button.move(450, 250)

        goto_button = QPushButton("50%", self)
        goto_button.resize(100, 200)
        goto_button.clicked.connect(self.submit_goto6)
        goto_button.move(550, 250)

        goto_button = QPushButton("60%", self)
        goto_button.resize(100, 200)
        goto_button.clicked.connect(self.submit_goto7)
        goto_button.move(150, 450)

        goto_button = QPushButton("70%", self)
        goto_button.resize(100, 200)
        goto_button.clicked.connect(self.submit_goto8)
        goto_button.move(250, 450)

        goto_button = QPushButton("80%", self)
        goto_button.resize(100, 200)
        goto_button.clicked.connect(self.submit_goto9)
        goto_button.move(350, 450)

        goto_button = QPushButton("90%", self)
        goto_button.resize(100, 200)
        goto_button.clicked.connect(self.submit_goto10)
        goto_button.move(450, 450)

        goto_button = QPushButton("100%", self)
        goto_button.resize(100, 200)
        goto_button.clicked.connect(self.submit_goto11)
        goto_button.move(550, 450)

        self.statusBar()

        self.setGeometry(300, 200, 800, 850)
        self.setWindowTitle("ARGripper GUI")
        self.show()

    def submit_goto1(self):
        self.gripper.goto_position(0.0, 100.0)

    def submit_goto2(self):
        self.gripper.goto_position(10.0, 100.0)

    def submit_goto3(self):
        self.gripper.goto_position(20.0, 100.0)

    def submit_goto4(self):
        self.gripper.goto_position(30.0, 100.0)

    def submit_goto5(self):
        self.gripper.goto_position(40.0, 100.0)

    def submit_goto6(self):
        self.gripper.goto_position(50.0, 100.0)

    def submit_goto7(self):
        self.gripper.goto_position(60.0, 100.0)

    def submit_goto8(self):
        self.gripper.goto_position(70.0, 100.0)

    def submit_goto9(self):
        self.gripper.goto_position(80.0, 100.0)

    def submit_goto10(self):
        self.gripper.goto_position(90.0, 100.0)

    def submit_goto11(self):
        self.gripper.goto_position(100.0, 100.0)


def main():
    rclpy.init(args=sys.argv)

    app = QApplication(sys.argv)
    timer = QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(100)

    ar_gripper_gui_node = Node("ar_gripper_gui")
    executor = MultiThreadedExecutor()
    executor.add_node(ar_gripper_gui_node)

    thread = Thread(target=executor.spin)
    ar_gripper_gui_node.get_logger().info("Starting ROS2 executor thread...")
    thread.start()

    try:
        _ = GripperGUI(ar_gripper_gui_node)
        sys.exit(app.exec_())
    finally:
        ar_gripper_gui_node.destroy_node()
        executor.shutdown()


if __name__ == "__main__":
    main()
