# coding: utf-8

from PyQt5.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout, QGraphicsScene, QGraphicsView, QPushButton, QMessageBox
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPainter
from roslibpy import Ros
from InteractiveGraphicsView import InteractiveGraphicsView
from qt_dotgraph.dot_to_qt import DotToQtGenerator
from qt_dotgraph.pydotfactory import PydotFactory
from dotcode import RosGraphDotcodeGenerator, NODE_NODE_GRAPH, NODE_TOPIC_ALL_GRAPH, NODE_TOPIC_GRAPH
import graph


class TopicGraphWidget(QWidget):
    def __init__(self, ros: Ros, parent=None):
        super(TopicGraphWidget, self).__init__(parent)
        self.ros_client = ros
        layout = QVBoxLayout()
        self._widget = QWidget()
        self._scene = QGraphicsScene()
        self._scene.setBackgroundBrush(Qt.white)
        self._graphics_view = InteractiveGraphicsView(self._widget)
        self._graphics_view.setScene(self._scene)
        self._graphics_view.setRenderHints(QPainter.Antialiasing | QPainter.HighQualityAntialiasing
                                           | QPainter.SmoothPixmapTransform | QPainter.TextAntialiasing)
        self._graphics_view.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        layout.addWidget(self._graphics_view)
        self._refresh_button = QPushButton('加载数据')
        self._refresh_button.setFixedWidth(100)
        self._refresh_button.clicked.connect(self._draw_graph_view)
        btn_layout = QHBoxLayout()
        btn_layout.addWidget(self._refresh_button)
        layout.addLayout(btn_layout)
        self.setLayout(layout)
        self._current_dotcode = None
        self.dot_to_qt = DotToQtGenerator()
        self.dotcode_factory = PydotFactory()
        self.dotcode_generator = RosGraphDotcodeGenerator(self.ros_client)
        self._graph = graph.Graph(self.ros_client)
        self._graph.set_master_stale(5.0)
        self._graph.set_node_stale(5.0)

    def _generate_dotcode(self):
        ns_filter = '/'
        topic_filter = '/'
        graph_mode = NODE_TOPIC_ALL_GRAPH
        orientation = 'LR'
        namespace_cluster = 2
        accumulate_actions = True
        hide_dead_end_topics = False
        hide_single_connection_topics = True
        quiet = True
        unreachable = True
        group_tf_nodes = True
        hide_tf_nodes = False
        group_image_nodes = True
        hide_dynamic_reconfigure = True

        return self.dotcode_generator.generate_dotcode(
            rosgraphinst=self._graph,
            ns_filter=ns_filter,
            topic_filter=topic_filter,
            graph_mode=graph_mode,
            hide_single_connection_topics=hide_single_connection_topics,
            hide_dead_end_topics=hide_dead_end_topics,
            cluster_namespaces_level=namespace_cluster,
            accumulate_actions=accumulate_actions,
            dotcode_factory=self.dotcode_factory,
            orientation=orientation,
            quiet=quiet,
            unreachable=unreachable,
            group_tf_nodes=group_tf_nodes,
            hide_tf_nodes=hide_tf_nodes,
            group_image_nodes=group_image_nodes,
            hide_dynamic_reconfigure=hide_dynamic_reconfigure)

    def _draw_graph_view(self):
        if self.ros_client is None:
            QMessageBox.critical(self, "错误", '未连接ROS')
            return
        if not self.ros_client.is_connected:
            QMessageBox.critical(self, "错误", '连接中断，请重新连接')
            return
        self._graph.update()
        self._current_dotcode = self._generate_dotcode()
        self._scene.clear()
        (nodes, edges) = self.dot_to_qt.dotcode_to_qt_items(self._current_dotcode,
                                                            highlight_level=3,
                                                            same_label_siblings=True,
                                                            scene=self._scene)
        self._scene.setSceneRect(self._scene.itemsBoundingRect())
        self._fit_in_view()

    def resizeEvent(self, e):
        self._fit_in_view()
        QWidget.resizeEvent(self, e)

    def _fit_in_view(self):
        self._graphics_view.fitInView(self._scene.itemsBoundingRect(), Qt.KeepAspectRatio)