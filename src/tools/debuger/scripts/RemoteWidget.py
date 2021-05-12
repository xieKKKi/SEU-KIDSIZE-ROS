# coding: utf-8

from PyQt5.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout, QMessageBox
from PyQt5.QtWidgets import QLabel, QSlider, QPushButton, QListWidget, QListWidgetItem
from PyQt5.QtCore import Qt, QTimer, QSize
from PyQt5.Qt import pyqtSignal
from roslibpy import Ros, Topic, Service, ServiceRequest


class RemoteSlider(QWidget):
    valueChanged = pyqtSignal()

    def __init__(self, name, minv, maxv, scale, parent=None):
        super(RemoteSlider, self).__init__(parent)
        layout = QHBoxLayout()
        nameLabel = QLabel(name)
        nameLabel.setFixedWidth(15 * len(name))
        layout.addWidget(nameLabel)
        self._slider = QSlider(Qt.Horizontal)
        self._slider.setRange(minv, maxv)
        layout.addWidget(self._slider)
        self._valueLabel = QLabel('0.0')
        self._valueLabel.setFixedWidth(50)
        layout.addWidget(self._valueLabel)
        self.setLayout(layout)
        self.scale = scale
        self._value = 0
        self._slider.valueChanged.connect(self._silder_changed)

    def value(self):
        return self._value

    def _silder_changed(self, val):
        self._value = float(val) / self.scale
        self._valueLabel.setText(str(self._value))
        self.valueChanged.emit()


class ToggleSwitch(QWidget):
    toggled = pyqtSignal()

    def __init__(self, name):
        super(ToggleSwitch, self).__init__()
        nameLabel = QLabel(name)
        nameLabel.setFixedWidth(12 * len(name))
        self._slider = QSlider(Qt.Horizontal)
        self._slider.setRange(0, 1)
        self._slider.setFixedWidth(50)
        layout = QHBoxLayout()
        layout.addStretch()
        layout.addWidget(nameLabel)
        layout.addWidget(self._slider)
        layout.addStretch()
        self.setLayout(layout)
        self._slider.valueChanged.connect(self.toggle)
        self._switch = False

    def switch(self):
        return self._switch

    def toggle(self):
        self._switch = True if self._slider.value() == 1 else False
        self.toggled.emit()


class HDevider(QLabel):
    def __init__(self):
        super(HDevider, self).__init__()
        self.setFixedHeight(2)
        self.setStyleSheet('QLabel{background:rgb(0,0,0);}')


class RemoteWidget(QWidget):
    def __init__(self, ros: Ros, parent=None):
        super(RemoteWidget, self).__init__(parent)
        self.ros_client = ros
        layout = QHBoxLayout()
        left_layout = QVBoxLayout()
        left_layout.addWidget(HDevider())
        self._step_slider = RemoteSlider('前进量', -40, 40, 1000.0)
        left_layout.addWidget(self._step_slider)
        self._lateral_slider = RemoteSlider('横移量', -30, 30, 1000.0)
        left_layout.addWidget(self._lateral_slider)
        self._turn_slider = RemoteSlider('偏转量', -20, 20, 1.0)
        left_layout.addWidget(self._turn_slider)
        self._run_button = QPushButton('执行')
        self._run_button.clicked.connect(self.on_run_clicked)
        blayout1 = QHBoxLayout()
        blayout1.addWidget(self._run_button)
        left_layout.addLayout(blayout1)
        left_layout.addWidget(HDevider())
        self._head_yaw_slider = RemoteSlider('头部方向', -120, 120, 1.0)
        self._head_yaw_slider.valueChanged.connect(self.on_head_task)
        left_layout.addWidget(self._head_yaw_slider)
        self._head_pitch_slider = RemoteSlider('头部俯仰', -90, 90, 1.0)
        self._head_pitch_slider.valueChanged.connect(self.on_head_task)
        left_layout.addWidget(self._head_pitch_slider)
        left_layout.addWidget(HDevider())
        led_layout = QHBoxLayout()
        self._led1_switch = ToggleSwitch('LED1')
        self._led1_switch.toggled.connect(self.on_led_task)
        self._led2_switch = ToggleSwitch('LED2')
        self._led2_switch.toggled.connect(self.on_led_task)
        self._imu_reset_btn = QPushButton('IMU重置')
        self._imu_reset_btn.clicked.connect(self.on_imu_reset)
        led_layout.addWidget(self._led1_switch)
        led_layout.addWidget(self._led2_switch)
        led_layout.addWidget(self._imu_reset_btn)
        left_layout.addLayout(led_layout)
        left_layout.addWidget(HDevider())
        layout.addLayout(left_layout)
        right_layout = QVBoxLayout()
        self._acts_widget = QListWidget()
        self._acts_widget.setStyleSheet("QListWidget::item { border-bottom: 1px solid black; }")
        self._acts_widget.itemDoubleClicked.connect(self.on_act_task)
        right_layout.addWidget(self._acts_widget)
        self._load_act_button = QPushButton('加载动作列表')
        self._load_act_button.clicked.connect(self.load_actions)
        right_layout.addWidget(self._load_act_button)
        layout.addLayout(right_layout)
        self.setLayout(layout)
        self._timer = QTimer()
        self._timer.timeout.connect(self.on_walk_task)
        self._timer.start(600)
        self._walk = False
        self._led_task = Topic(self.ros_client, '/task/led', 'common/LedTask')
        self._led_task.advertise()
        self._head_task = Topic(self.ros_client, '/task/head', 'common/HeadTask')
        self._head_task.advertise()
        self._body_task = Topic(self.ros_client, '/task/body', 'common/BodyTask')
        self._body_task.advertise()

    def check_connect(self):
        if self.ros_client is None:
            QMessageBox.critical(self, "错误", '未连接ROS')
            return False
        if not self.ros_client.is_connected:
            QMessageBox.critical(self, "错误", '连接中断，请重新连接')
            if self._walk:
                self._run_button.clicked.emit(True)
            return False
        return True

    def load_actions(self):
        self._acts_widget.clear()
        if not self.check_connect():
            return
        try:
            service = Service(self.ros_client, '/get_actions', 'common/GetActions')
            result = service.call(ServiceRequest({}))
            actions = result['actions']
            self._acts_widget.clear()
            for action in actions:
                litem = QListWidgetItem()
                aitem = QLabel(action)
                aitem.setFixedHeight(30)
                aitem.setStyleSheet('QLabel{margin-left: 5px;}')
                aitem.show()
                self._acts_widget.addItem(litem)
                self._acts_widget.setItemWidget(litem, aitem)
                litem.setSizeHint(QSize(aitem.rect().width(), aitem.rect().height()))
        except Exception as e:
            QMessageBox.critical(self, "错误", e.args[0])

    def on_run_clicked(self):
        if not self.check_connect():
            return
        self._walk = not self._walk
        self._run_button.setText('停止' if self._walk else '执行')

    def on_imu_reset(self):
        if not self.check_connect():
            return
        try:
            service = Service(self.ros_client, '/imu_reset', 'std_srvs/Empty')
            service.call(ServiceRequest({}))
        except Exception as e:
            QMessageBox.critical(self, "错误", e.args[0])

    def on_led_task(self):
        if not self.check_connect():
            return
        self._led_task.publish({'led1': self._led1_switch.switch(),
                                'led2': self._led2_switch.switch()})

    def on_head_task(self):
        if not self.check_connect():
            return
        self._head_task.publish({'mode': 3,
                                 'yaw': self._head_yaw_slider.value(),
                                 'pitch': self._head_pitch_slider.value()})

    def on_walk_task(self):
        if self.ros_client is None or not self.ros_client.is_connected:
            return
        if self._walk and self.ros_client.is_connected:
            self._body_task.publish({'type': 1,
                                     'actname': '',
                                     'count': 2,
                                     'step': self._step_slider.value(),
                                     'lateral': self._lateral_slider.value(),
                                     'turn': self._turn_slider.value()})

    def on_act_task(self):
        # double click on action!
        if not self.check_connect():
            return
        item = self._acts_widget.currentItem()
        if item is None:
            return
        aitem = self._acts_widget.itemWidget(item)
        aitem.__class__ = QLabel
        action = aitem.text()
        if self._walk:
            self.on_run_clicked()
        self._body_task.publish({'type': 2,
                                 'actname': action,
                                 'count': 1,
                                 'step': 0,
                                 'lateral': 0,
                                 'turn': 0})


