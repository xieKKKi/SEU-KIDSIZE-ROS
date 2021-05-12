# coding: utf-8

from PyQt5.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout, QPushButton
from PyQt5.QtWidgets import QListWidget, QListWidgetItem, QLabel
from PyQt5.QtCore import QSize, Qt, pyqtSignal
from roslibpy import Ros, Topic


class LogItem(QWidget):
    def __init__(self, name: str, value, parent=None):
        super(LogItem, self).__init__(parent)
        self.tp = type(value)
        self.nameLabel = QLabel(name)
        self.nameLabel.setFixedSize(200, 20)
        self.valueLabel = QLabel(str(value))
        layout = QHBoxLayout()
        layout.addWidget(self.nameLabel)
        layout.addWidget(self.valueLabel)
        self.setLayout(layout)


class LogWidget(QWidget):
    recv_msg = pyqtSignal(dict)

    def __init__(self, ros: Ros, parent=None):
        super(LogWidget, self).__init__(parent)
        self.setMinimumSize(100, 100)
        self.listWidget = QListWidget()
        self.listWidget.setStyleSheet("QListWidget::item { border-bottom: 1px solid black; }")
        layout = QVBoxLayout()
        layout.addWidget(self.listWidget)
        self._clear_button = QPushButton('清除记录')
        self._clear_button.setFixedWidth(100)
        self._clear_button.clicked.connect(self.clear_logs)
        btn_layout = QHBoxLayout()
        btn_layout.addWidget(self._clear_button)
        layout.addLayout(btn_layout)
        self.setLayout(layout)
        self.ros_client = ros
        self._log_topic = Topic(self.ros_client, '/rosout', 'rosgraph_msgs/Log')
        self._log_topic.subscribe(self.on_log)
        self.recv_msg.connect(self.on_recv_msg)

    def _insert_item_front(self, name: str, value):
        pitem = LogItem(name, value, self.listWidget)
        litem = QListWidgetItem()
        pitem.show()
        self.listWidget.insertItem(0, litem)
        self.listWidget.setItemWidget(litem, pitem)
        litem.setSizeHint(QSize(pitem.rect().width(), pitem.rect().height()))

    def on_log(self, msg):
        self.recv_msg.emit(msg)

    def on_recv_msg(self, msg):
        self._insert_item_front(msg['name'], msg['msg'])

    def clear_logs(self):
        self.listWidget.clear()
