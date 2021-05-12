# coding: utf-8

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QHeaderView, QTableWidgetItem, QAbstractItemView
from PyQt5.QtWidgets import QPushButton, QTableWidget, QMessageBox
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QBrush, QColor
from roslibpy import Ros


class ServiceTableWidget(QWidget):
    def __init__(self, ros: Ros, parent=None):
        super(ServiceTableWidget, self).__init__(parent)
        self.ros_client = ros
        self.tableWidget = QTableWidget()
        self.tableWidget.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self._load_btn = QPushButton('加载数据')
        self._load_btn.clicked.connect(self._load)
        btn_layout = QHBoxLayout()
        btn_layout.addWidget(self._load_btn)
        layout = QVBoxLayout()
        layout.addWidget(self.tableWidget)
        layout.addLayout(btn_layout)
        self.setLayout(layout)

    def _load(self):
        if self.ros_client is None:
            QMessageBox.critical(self, "错误", '未连接ROS')
            return
        if not self.ros_client.is_connected:
            QMessageBox.critical(self, "错误", '连接中断，请重新连接')
            return
        nodes = self.ros_client.get_nodes()
        node_services = {}
        for node in nodes:
            if node.find('rosapi') < 0 and node.find('rosout') < 0:
                details = self.ros_client.get_node_details(node)
                node_services[node] = details['services']
        all_services = self.ros_client.get_services()
        services = ['']
        for service in all_services:
            if service.find('rosapi') < 0 and service.find('rosout') < 0 \
                    and service.find('get_loggers') < 0 and service.find('set_logger_level') < 0:
                services.append(service)
        self.tableWidget.setColumnCount(len(services))
        self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tableWidget.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tableWidget.setHorizontalHeaderLabels(services)
        self.tableWidget.setRowCount(len(node_services))
        rows = 0
        for node in node_services.keys():
            n_item = QTableWidgetItem(node)
            self.tableWidget.setItem(rows, 0, n_item)
            self.tableWidget.item(rows, 0).setTextAlignment(Qt.AlignCenter)
            for service in node_services[node]:
                try:
                    col = services.index(service)
                    self.tableWidget.setItem(rows, col, QTableWidgetItem(''))
                    self.tableWidget.item(rows, col).setBackground(QBrush(QColor(0, 0x99, 0)))
                except ValueError:
                    pass
            rows = rows + 1
