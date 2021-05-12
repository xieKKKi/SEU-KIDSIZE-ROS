# coding: utf-8

from PyQt5.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout, QPushButton, QMessageBox
from PyQt5.QtWidgets import QListWidget, QListWidgetItem, QLabel, QInputDialog, QLineEdit
from PyQt5.QtCore import QSize, Qt
from PyQt5.QtGui import QStandardItemModel
from roslibpy import Ros


class ParamsItem(QWidget):
    def __init__(self, name: str, value, parent=None):
        super(ParamsItem, self).__init__(parent)
        self.tp = type(value)
        self.nameLabel = QLabel(name)
        self.nameLabel.setFixedWidth(200)
        self.valueLabel = QLabel(str(value))
        layout = QHBoxLayout()
        layout.addWidget(self.nameLabel)
        layout.addWidget(self.valueLabel)
        self.setLayout(layout)


class ParamsWidget(QWidget):
    def __init__(self, ros: Ros, parent=None):
        super(ParamsWidget, self).__init__(parent)
        self.listWidget = QListWidget()
        self.listWidget.setStyleSheet("QListWidget::item { border-bottom: 1px solid black; }")
        self.listWidget.itemDoubleClicked.connect(self._on_item_double_click)
        layout = QVBoxLayout()
        layout.addWidget(self.listWidget)
        self._refresh_button = QPushButton('加载数据')
        self._refresh_button.setFixedWidth(100)
        self._refresh_button.clicked.connect(self.load_params)
        btn_layout = QHBoxLayout()
        btn_layout.addWidget(self._refresh_button)
        layout.addLayout(btn_layout)
        self.setLayout(layout)
        self.ros_client = ros

    def _on_item_double_click(self, item):
        if not self.check_connect():
            return
        pitem = self.listWidget.itemWidget(item)
        pitem.__class__ = ParamsItem
        name = pitem.nameLabel.text()
        value = pitem.valueLabel.text()
        tp = pitem.tp
        text, ok_pressed = QInputDialog.getText(self, '参数修改', name, QLineEdit.Normal, value)
        if ok_pressed and text != '':
            if tp is type(1):
                value = int(text)
            elif tp is type(True):
                value = bool(text)
            elif tp == type(1.0):
                value = float(text)
            else:
                value = text
            self.ros_client.set_param(name, value)
        self.load_params()

    def check_connect(self):
        if self.ros_client is None:
            QMessageBox.critical(self, "错误", '未连接ROS')
            return False
        if not self.ros_client.is_connected:
            QMessageBox.critical(self, "错误", '连接中断，请重新连接')
            return False
        return True

    def _add_item(self, name: str, value):
        pitem = ParamsItem(name, value, self.listWidget)
        litem = QListWidgetItem()
        pitem.show()
        self.listWidget.addItem(litem)
        self.listWidget.setItemWidget(litem, pitem)
        litem.setSizeHint(QSize(pitem.rect().width(), pitem.rect().height()))

    def load_params(self):
        if not self.check_connect():
            return
        names = self.ros_client.get_params()
        self.listWidget.clear()
        for name in names:
            name = str(name)
            if name.find('rosbridge') < 0 and name.find('roslaunch') < 0 \
                    and name.find('rosapi') < 0 and name.find('rosversion') < 0 \
                    and name.find('rosdistro') < 0:
                value = self.ros_client.get_param(name)
                self._add_item(name, value)
