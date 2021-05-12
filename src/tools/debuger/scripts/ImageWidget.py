# coding: utf-8

from PyQt5.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout, QMessageBox
from PyQt5.QtWidgets import QLabel, QPushButton, QComboBox, QCheckBox
from PyQt5.QtGui import QPaintEvent, QMouseEvent, QImage, QPainter, QPixmap, QPen
from PyQt5.QtCore import QSize, Qt, QRect, pyqtSignal
from roslibpy import Ros, Topic, Service, ServiceRequest
import base64
import os
import datetime

IMAGE_SAVE_DIR = 'images/'


class ImageLabel(QLabel):
    shot = pyqtSignal(QRect)

    def __init__(self, w=640, h=480):
        super(ImageLabel, self).__init__()
        self.setFixedSize(w, h)
        self.setStyleSheet("QLabel{background:black}")
        self._drawing = False
        self._image = None
        self._shot_rect = QRect()
        self._start_pos = None
        self._end_pos = None

    def set_image(self, name):
        self._image = QImage()
        self._image.load(name)
        self.update()

    def mousePressEvent(self, ev: QMouseEvent) -> None:
        if ev.button() == Qt.LeftButton:
            self._drawing = True
            self._start_pos = ev.pos()

    def mouseMoveEvent(self, ev: QMouseEvent) -> None:
        if self._drawing:
            self._end_pos = ev.pos()
            self._shot_rect = QRect(self._start_pos, self._end_pos)
            self.update()

    def mouseReleaseEvent(self, ev: QMouseEvent) -> None:
        if ev.button() == Qt.LeftButton:
            self.shot.emit(self._shot_rect)
            self._drawing = False
            self._shot_rect.setWidth(0)
            self._shot_rect.setHeight(0)

    def paintEvent(self, a0: QPaintEvent) -> None:
        painter = QPainter(self)
        if self._image is not None:
            pixmap = QPixmap.fromImage(self._image)
            painter.drawPixmap(0, 0, self._image.width(), self._image.height(), pixmap)
        if self._drawing:
            painter.setPen(QPen(Qt.red, 4, Qt.SolidLine, Qt.FlatCap))
            painter.drawRect(self._shot_rect)
            painter.setPen(QPen(Qt.red, 2, Qt.SolidLine, Qt.FlatCap))
            painter.drawLine(self._shot_rect.topLeft(), self._shot_rect.bottomRight())
            painter.drawLine(self._shot_rect.topRight(), self._shot_rect.bottomLeft())


class ImageWidget(QWidget):
    on_image = pyqtSignal(str)

    def __init__(self, ros: Ros, parent=None):
        super(ImageWidget, self).__init__(parent)
        if not os.path.exists(IMAGE_SAVE_DIR):
            os.mkdir(IMAGE_SAVE_DIR)
        self.ros_client = ros
        layout = QVBoxLayout()
        self._image_label = ImageLabel()
        self._image_label.shot.connect(self.on_shot)
        layout.addWidget(self._image_label)
        btn_layout = QHBoxLayout()
        self._subscribe_btn = QPushButton('订阅图像')
        self._subscribe_btn.clicked.connect(self.subscribe_image)
        self._type_combo_box = QComboBox()
        self._type_combo_box.addItems(['None', 'Origin', 'Result'])
        self._type_combo_box.currentIndexChanged.connect(self.on_type_changed)
        self._func_combo_box = QComboBox()
        self._func_combo_box.addItems(['function1', 'function2'])
        self._save_btn = QPushButton('保存')
        self._save_btn.clicked.connect(self.on_save_clicked)
        self._auto_save_check_box = QCheckBox('自动保存')
        self._auto_save_check_box.toggled.connect(self.on_auto_save_check)

        btn_layout.addWidget(self._subscribe_btn)
        btn_layout.addWidget(self._type_combo_box)
        btn_layout.addWidget(self._func_combo_box)
        btn_layout.addWidget(self._save_btn)
        btn_layout.addWidget(self._auto_save_check_box)
        layout.addLayout(btn_layout)
        self.setLayout(layout)
        self._save = False
        self._auto_save = False
        self._subscribe = False
        self._image_topic = Topic(self.ros_client, '/result/vision/compressed', 'sensor_msgs/CompressedImage')
        self.on_image.connect(self.on_recv_image)
        self._last_seq = 0

    def check_connect(self):
        if not self.ros_client.is_connected:
            QMessageBox.critical(self, "错误", '连接中断，请重新连接')
            return False
        return True

    def subscribe_image(self):
        if not self.check_connect():
            return
        self._subscribe = not self._subscribe
        if self._subscribe:
            self._image_topic.subscribe(self.recv_image)
            self._subscribe_btn.setText('取消订阅')
            t = self.ros_client.get_param('image')
            if t is not None:
                self._type_combo_box.setCurrentIndex(t)
        else:
            self._image_topic.unsubscribe()
            self._subscribe_btn.setText('订阅图像')

    def recv_image(self, msg):
        base64_bytes = msg['data'].encode('ascii')
        image_bytes = base64.b64decode(base64_bytes)
        fmt = msg['format']
        seq = msg['header']['seq']
        name = IMAGE_SAVE_DIR + 'temp.{}'.format(fmt)
        if self._auto_save or self._save:
            if seq - self._last_seq > 100 or self._save:
                self._save = False
                name = IMAGE_SAVE_DIR + datetime.datetime.now().strftime('%Y%m%d%H%M%S') + '.' + fmt
                self._last_seq = seq

        with open(name, 'wb') as image_file:
            image_file.write(image_bytes)
        self.on_image.emit(name)

    def on_recv_image(self, name):
        self._image_label.set_image(name)

    def on_auto_save_check(self, val):
        self._auto_save = val

    def on_save_clicked(self):
        self._save = True

    def on_shot(self, rect: QRect):
        if not self.check_connect():
            return
        try:
            service = Service(self.ros_client, '/debug/image/snap', 'common/ImageSnap')
            result = service.call(ServiceRequest({'type': self._func_combo_box.currentIndex(),
                                         'info': {
                                             'x': rect.topLeft().x(), 'y': rect.topLeft().y(),
                                             'w': rect.width(), 'h': rect.height()
                                         }}))
        except Exception as e:
            QMessageBox.critical(self, "错误", e.args[0])

    def on_type_changed(self, idx):
        if not self.check_connect():
            return
        try:
            service = Service(self.ros_client, '/setting/sendtype', 'common/SetInt')
            service.call(ServiceRequest({'number': idx}))
        except Exception as e:
            QMessageBox.critical(self, "错误", e.args[0])
