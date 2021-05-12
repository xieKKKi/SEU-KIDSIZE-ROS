# coding: utf-8

from PyQt5.QtWidgets import QMainWindow, QMessageBox, QHBoxLayout, QVBoxLayout, QSplitter
from PyQt5.QtWidgets import QWidget, QLabel, QGroupBox, QInputDialog, QLineEdit, QAction
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5 import QtGui
from ParamsWidget import ParamsWidget
from TopicGraphWidget import TopicGraphWidget
from ServiceTableWidget import ServiceTableWidget
from LogWidget import LogWidget
from RemoteWidget import RemoteWidget
from ImageWidget import ImageWidget
from roslibpy import Ros, Service, ServiceRequest
import os


class MainWindow(QMainWindow):
    on_run = pyqtSignal(dict)
    disconnected = pyqtSignal()

    def __init__(self):
        super(MainWindow, self).__init__()
        self.host, ok_pressed = QInputDialog.getText(self, '连接', '机器人地址', QLineEdit.Normal, '192.168.0.101')
        if not ok_pressed or len(self.host) < 7:
            exit(0)
        try:
            self.ros_client = Ros(self.host, 9090)
            self.ros_client.run(3)
            self.ros_client.on('close', self.on_lost_connect)
        except Exception as e:
            QMessageBox.critical(self, "错误", e.args[0])
            exit(0)

        self.setWindowTitle('调试器')
        self.statusBar().setSizeGripEnabled(False)
        self.statusBar().setStyleSheet('border: 1px solid black;')

        self._toolMenu = self.menuBar().addMenu('工具')
        self._srv_action = QAction('Service表', self._toolMenu)
        self._srv_action.setCheckable(True)
        self._srv_action.toggled.connect(self.on_srv_table_widget)
        self._topics_action = QAction('Topics图', self._toolMenu)
        self._topics_action.setCheckable(True)
        self._topics_action.toggled.connect(self.on_topics_widget)
        self._remote_action = QAction('遥控', self._toolMenu)
        self._remote_action.setCheckable(True)
        self._remote_action.toggled.connect(self.on_remote_widget)
        self._logs_action = QAction('日志', self._toolMenu)
        self._logs_action.setCheckable(True)
        self._logs_action.toggled.connect(self.on_log_widget)
        self._image_action = QAction('图像', self._toolMenu)
        self._image_action.setCheckable(True)
        self._image_action.toggled.connect(self.on_image_widget)
        self._params_action = QAction('参数', self._toolMenu)
        self._params_action.setCheckable(True)
        self._params_action.toggled.connect(self.on_params_widget)
        self._toolMenu.addAction(self._srv_action)
        self._toolMenu.addAction(self._topics_action)
        self._toolMenu.addAction(self._remote_action)
        self._toolMenu.addAction(self._logs_action)
        self._toolMenu.addAction(self._image_action)
        self._toolMenu.addAction(self._params_action)
        self.menuBar().addSeparator()
        self._act_debug_action = self.menuBar().addAction('动作调试器')
        self._act_debug_action.triggered.connect(self.on_act_debug)
        self.menuBar().addSeparator()
        self._state_monitor_action = self.menuBar().addAction('状态监测器')
        self._state_monitor_action.triggered.connect(self.on_state_monitor)
        self.hostLabel = QLabel(self.host)
        self.hostLabel.setText(self.host)
        self.statusBar().addWidget(self.hostLabel)

        self.mainWidget = QWidget()
        self.main_layout = QVBoxLayout()
        self.main_splitter = QSplitter(Qt.Horizontal, self)
        self.left_splitter = QSplitter(Qt.Vertical, self.main_splitter)
        self.middle_splitter = QSplitter(Qt.Vertical, self.main_splitter)
        self.right_splitter = QSplitter(Qt.Vertical, self.main_splitter)

        self.main_splitter.addWidget(self.left_splitter)
        self.main_splitter.addWidget(self.middle_splitter)
        self.main_splitter.addWidget(self.right_splitter)
        self.boxqss = '''QGroupBox { border: 2px solid black;
                                        border-radius: 5px;
                                        margin-top:1ex; } 
                             QGroupBox::title {
                                subcontrol-origin: margin;
                                subcontrol-position: top center; 
                                padding: 0 3px; }'''
        self.main_layout.addWidget(self.main_splitter)
        self.mainWidget.setLayout(self.main_layout)
        self.setCentralWidget(self.mainWidget)

        self.action_debug_client = None
        self.on_run.connect(self.run_add_angles)
        self.disconnected.connect(self.on_disconnected)
        self.srvWidget = None
        self.graphWidget = None
        self.remoteWidget = None
        self.logWidget = None
        self.imageWidget = None
        self.paramsWidget = None
        self.on_srv_table_widget()
        self.on_topics_widget()
        self.on_remote_widget()
        self.on_log_widget()
        self.on_image_widget()
        self.on_params_widget()

    def on_recv_action(self, req, res):
        self.on_run.emit(req)
        return True

    def run_add_angles(self, req):
        try:
            run_service = Service(self.ros_client, '/add_angles', 'common/AddAngles')
            run_service.call(ServiceRequest(req))
        except Exception as e:
            QMessageBox.critical(self, "错误", e.args[0])

    def on_act_debug(self):
        if not self.ros_client.is_connected:
            QMessageBox.critical(self, "错误", '尚未连接到ROS')
            return
        if self.host == '127.0.0.1' or self.host == 'localhost':
            self.action_debug_client = self.ros_client
            cmd = 'rosrun action action_debuger'
        else:
            cmd = 'roslaunch start start_action_debug_robot.launch'
            try:
                if self.action_debug_client is None:
                    print("run action_debug_client at 127.0.0.1:9090")
                    self.action_debug_client = Ros('127.0.0.1', 9090)
                    self.action_debug_client.run()
                elif not self.action_debug_client.is_connected:
                    print("connecting action debug client")
                    self.action_debug_client.connect()
            except Exception as e:
                QMessageBox.critical(self, "错误", '无法连接到本地调试器 %s' % str(e))
                return

        act_service = Service(self.action_debug_client, '/debug/action/run', 'common/AddAngles')
        act_service.advertise(self.on_recv_action)
        os.popen(cmd)

    def on_state_monitor(self):
        cmd = 'rosrun team_monitor  team_monitor '
        os.popen(cmd)

    def on_lost_connect(self, proto):
        self.disconnected.emit()

    def on_disconnected(self):
        QMessageBox.critical(self, "错误", '连接已断开')
        self.close()

    def on_srv_table_widget(self, show=None):
        if show is None:
            groupbox = QGroupBox('Service表')
            groupbox.setStyleSheet(self.boxqss)
            layout = QVBoxLayout()
            self.srvWidget = ServiceTableWidget(self.ros_client)
            layout.addWidget(self.srvWidget)
            groupbox.setLayout(layout)
            groupbox.hide()
            self.left_splitter.insertWidget(0, groupbox)
        elif show:
            self.left_splitter.widget(0).show()
        else:
            self.left_splitter.widget(0).hide()
        self.adjustSize()

    def on_topics_widget(self, show=None):
        if show is None:
            groupbox = QGroupBox('Topic拓扑图')
            groupbox.setStyleSheet(self.boxqss)
            layout = QVBoxLayout()
            self.graphWidget = TopicGraphWidget(self.ros_client)
            layout.addWidget(self.graphWidget)
            groupbox.setLayout(layout)
            groupbox.hide()
            self.left_splitter.insertWidget(1, groupbox)
        elif show:
            self.left_splitter.widget(1).show()
        else:
            self.left_splitter.widget(1).hide()
        self.adjustSize()

    def on_remote_widget(self, show=None):
        if show is None:
            groupbox = QGroupBox('遥控')
            groupbox.setStyleSheet(self.boxqss)
            layout = QVBoxLayout()
            self.remoteWidget = RemoteWidget(self.ros_client)
            layout.addWidget(self.remoteWidget)
            groupbox.setLayout(layout)
            groupbox.hide()
            self.middle_splitter.insertWidget(0, groupbox)
        elif show:
            self.middle_splitter.widget(0).show()
        else:
            self.middle_splitter.widget(0).hide()
        self.adjustSize()

    def on_log_widget(self, show=None):
        if show is None:
            groupbox = QGroupBox('运行日志')
            groupbox.setStyleSheet(self.boxqss)
            layout = QVBoxLayout()
            self.logWidget = LogWidget(self.ros_client)
            layout.addWidget(self.logWidget)
            groupbox.setLayout(layout)
            groupbox.hide()
            self.middle_splitter.insertWidget(1, groupbox)
        elif show:
            self.middle_splitter.widget(1).show()
        else:
            self.middle_splitter.widget(1).hide()
        self.adjustSize()

    def on_image_widget(self, show=None):
        if show is None:
            groupbox = QGroupBox('在线图像')
            groupbox.setStyleSheet(self.boxqss)
            layout = QVBoxLayout()
            self.imageWidget = ImageWidget(self.ros_client)
            layout.addWidget(self.imageWidget)
            groupbox.setLayout(layout)
            groupbox.hide()
            self.right_splitter.insertWidget(0, groupbox)
        elif show:
            self.right_splitter.widget(0).show()
        else:
            self.right_splitter.widget(0).hide()
        self.adjustSize()

    def on_params_widget(self, show=None):
        if show is None:
            groupbox = QGroupBox('参数')
            groupbox.setStyleSheet(self.boxqss)
            layout = QVBoxLayout()
            self.paramsWidget = ParamsWidget(self.ros_client)
            layout.addWidget(self.paramsWidget)
            groupbox.setLayout(layout)
            groupbox.hide()
            self.right_splitter.insertWidget(1, groupbox)
        elif show:
            self.right_splitter.widget(1).show()
        else:
            self.right_splitter.widget(1).hide()
        self.adjustSize()

    def closeEvent(self, a0: QtGui.QCloseEvent) -> None:
        if self.ros_client is not None:
            self.ros_client.terminate()
