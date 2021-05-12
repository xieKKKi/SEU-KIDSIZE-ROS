#!/usr/bin/python3
# coding: utf-8

from PyQt5.QtWidgets import QApplication
from MainWindow import MainWindow
import sys
import logging
# Configure logging to high verbosity (DEBUG)
# fmt = '%(asctime)s %(levelname)8s: %(message)s'
# logging.basicConfig(format=fmt, level=logging.DEBUG)
# log = logging.getLogger(__name__)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    foo = MainWindow()
    foo.show()
    sys.exit(app.exec())