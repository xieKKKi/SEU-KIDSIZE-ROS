#!/usr/bin/env python3
# coding: utf-8

import sys
import os
import json
import common
import config
from PyQt5 import QtWidgets, QtCore, QtGui
import SSH


class OptionWidget(QtWidgets.QWidget):
    trans = QtCore.pyqtSignal(int, int)

    def __init__(self, host):
        super(OptionWidget, self).__init__()
        self._host = host
        self._ssh = None
        self._connected = False
        self._shell = None
        self._init_ui()
        self.hostEdit.setText(self._host)
        self.setOptEnable(False)
        self._recv_threads = []

    def _init_ui(self):
        mainLayout = QtWidgets.QVBoxLayout()
        upLayout = QtWidgets.QVBoxLayout()
        hostLayout = QtWidgets.QHBoxLayout()
        self.hostEdit = QtWidgets.QLineEdit()
        self._connBtn = QtWidgets.QPushButton('连接')
        self._connBtn.clicked.connect(self.proc_btn_connect)
        hostLayout.addWidget(self.hostEdit)
        hostLayout.addWidget(self._connBtn)
        upLayout.addLayout(hostLayout)
        files = os.listdir(config.local_root + '/src')
        srcGroup = QtWidgets.QGroupBox("代码文件夹")
        srcGroup.setStyleSheet('''QGroupBox { border: 2px solid black;
                                                border-radius: 5px;
                                                margin-top:1ex; } 
                                     QGroupBox::title {
                                        subcontrol-origin: margin;
                                        subcontrol-position: top center; 
                                        padding: 0 3px; }''')
        gboxLayout = QtWidgets.QVBoxLayout()
        self._srcCheckBoxes = []
        slayout = None
        i = 0
        for f in files:
            if i % 2 == 0:
                slayout = QtWidgets.QHBoxLayout()
                gboxLayout.addLayout(slayout)
            if not os.path.isdir(config.local_root + '/src/' + f):
                continue
            if f.find('tools') >= 0:
                label = QtWidgets.QLabel(f)
                label.setFixedWidth(150)
                slayout.addWidget(label)
            else:
                box = QtWidgets.QCheckBox(f)
                box.setFixedWidth(150)
                self._srcCheckBoxes.append(box)
                slayout.addWidget(box)
            i = i + 1
        srcGroup.setLayout(gboxLayout)
        upLayout.addWidget(srcGroup)
        btnLayout = QtWidgets.QHBoxLayout()
        self._uploadSrcsBtn = QtWidgets.QPushButton('上传代码')
        self._uploadSrcsBtn.clicked.connect(self.proc_btn_upload_srcs)
        btnLayout.addWidget(self._uploadSrcsBtn)
        self._buildBtn = QtWidgets.QPushButton('编译')
        self._buildBtn.clicked.connect(self.proc_btn_build)
        btnLayout.addWidget(self._buildBtn)
        upLayout.addLayout(btnLayout)
        btnLayout = QtWidgets.QHBoxLayout()
        self._runDebugBtn = QtWidgets.QPushButton('启动调试服务器')
        self._runDebugBtn.clicked.connect(self.proc_btn_run_debug)
        btnLayout.addWidget(self._runDebugBtn)
        self._runRobotBtn = QtWidgets.QPushButton('启动机器人')
        self._runRobotBtn.clicked.connect(self.proc_btn_run_robot)
        btnLayout.addWidget(self._runRobotBtn)
        upLayout.addLayout(btnLayout)
        btnLayout = QtWidgets.QHBoxLayout()
        self._runRemoteBtn = QtWidgets.QPushButton('以遥控模式启动')
        self._runRemoteBtn.clicked.connect(self.proc_btn_run_remote)
        btnLayout.addWidget(self._runRemoteBtn)
        self._runNoVision = QtWidgets.QPushButton('无视觉启动')
        self._runNoVision.clicked.connect(self.proc_btn_run_no_vision)
        btnLayout.addWidget(self._runNoVision)
        upLayout.addLayout(btnLayout)
        btnLayout = QtWidgets.QHBoxLayout()
        self._runRebootBtn = QtWidgets.QPushButton('重启')
        self._runRobotBtn.clicked.connect(self.proc_btn_reboot)
        btnLayout.addWidget(self._runRebootBtn)
        self._runPoweroffVision = QtWidgets.QPushButton('关机')
        self._runPoweroffVision.clicked.connect(self.proc_btn_power_off)
        btnLayout.addWidget(self._runPoweroffVision)
        upLayout.addLayout(btnLayout)
        mainLayout.addLayout(upLayout)
        self._processBar = QtWidgets.QProgressBar()
        self._processBar.setRange(0, 100)
        mainLayout.addWidget(self._processBar)
        self.setLayout(mainLayout)
        self.trans.connect(self.on_tran)

    def setOptEnable(self, e):
        self._uploadSrcsBtn.setEnabled(e)
        self._buildBtn.setEnabled(e)
        self._runRobotBtn.setEnabled(e)
        self._runDebugBtn.setEnabled(e)
        self._runRemoteBtn.setEnabled(e)
        self._runNoVision.setEnabled(e)
        self._runRebootBtn.setEnabled(e)
        self._runPoweroffVision.setEnabled(e)

    def proc_btn_connect(self):
        if not self._connected:
            try:
                self._ssh = SSH.SSH(self.hostEdit.text(), config.username, config.password)
                self._shell = self._ssh.create_shell()
                self._connected = True
                self._connBtn.setText('断开连接')
                self.setOptEnable(True)
                print(self._ssh.get_remote_time())
                self._ssh.set_remote_time()
            except Exception as e:
                print(e.args)
        else:
            self._ssh.close()
            self._connected = False
            self._connBtn.setText('连接')
            self._ssh = None
            self._shell = None
            self.setOptEnable(False)

    def proc_btn_upload_srcs(self):
        srcs = []
        for box in self._srcCheckBoxes:
            if box.isChecked():
                srcs.append(box.text())
        if len(srcs) == 0:
            return
        target = 'srcs.tar.gz'
        cmd = 'cd {root}/src; tar czvf {root}/{target} {srcs}'.format(root=config.local_root,
                                                                      target=target, srcs=' '.join(srcs))
        ret, out, err = common.run_cmd(cmd)
        if not ret:
            return
        self._ssh.upload(config.local_root + '/' + target, config.remote_root + '/' + target, self.transport_status)
        self._shell.exec_command('cd {}'.format(config.remote_root))
        self._shell.exec_command('tar -zxvf {} -C src/'.format(target))

    def proc_btn_build(self):
        cmdlist = ['cd {}'.format(config.remote_root),
                   'source /home/nvidia/.bashrc',
                   'catkin_make --pkg common',
                   'catkin_make']
        self._recv_threads.append(self._ssh.exec_command(cmdlist))

    def proc_btn_run_debug(self):
        cmdlist = ['cd {}'.format(config.remote_root),
                   'source /home/nvidia/.bashrc',
                   'source devel/setup.bash',
                   'roslaunch start start_debug_server.launch']
        self._recv_threads.append(self._ssh.exec_command(cmdlist))

    def proc_btn_run_robot(self):
        cmdlist = ['cd {}'.format(config.remote_root),
                   'source /home/nvidia/.bashrc',
                   'source devel/setup.bash',
                   'roslaunch start start_robot.launch']
        self._recv_threads.append(self._ssh.exec_command(cmdlist))

    def proc_btn_run_remote(self):
        cmdlist = ['cd {}'.format(config.remote_root),
                   'source /home/nvidia/.bashrc',
                   'source devel/setup.bash',
                   'roslaunch start start_robot_remote.launch']
        self._recv_threads.append(self._ssh.exec_command(cmdlist))

    def proc_btn_run_no_vision(self):
        cmdlist = ['cd {}'.format(config.remote_root),
                   'source /home/nvidia/.bashrc',
                   'source devel/setup.bash',
                   'roslaunch start start_robot_without_vision.launch']
        self._recv_threads.append(self._ssh.exec_command(cmdlist))

    def proc_btn_reboot(self):
        shell = self._ssh.create_shell()
        shell.exec_command('reboot')

    def proc_btn_power_off(self):
        shell = self._ssh.create_shell()
        shell.exec_command('poweroff')

    def transport_status(self, transd, total):
        self.trans.emit(transd, total)

    def on_tran(self, transd, total):
        self._processBar.setValue(int(float(transd) / total * 100))
        QtWidgets.QApplication.processEvents()

    def terminate(self):
        for t in self._recv_threads:
            t.terminate()
            t.join()


class EasyWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(EasyWindow, self).__init__()
        self.setWindowTitle('快捷使用')
        self.tabWidget = QtWidgets.QTabWidget()
        self.setCentralWidget(self.tabWidget)
        self._widgets = []
        players = common.get_config(config.conf_file, 'players')
        for player_id in players.keys():
            widget = OptionWidget(players[player_id]['address'])
            self.tabWidget.addTab(widget, '{}号机器人'.format(int(player_id)))
            self._widgets.append(widget)

    def closeEvent(self, a0: QtGui.QCloseEvent) -> None:
        for widget in self._widgets:
            widget.terminate()


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    app.processEvents()
    window = EasyWindow()
    window.show()
    sys.exit(app.exec_())
