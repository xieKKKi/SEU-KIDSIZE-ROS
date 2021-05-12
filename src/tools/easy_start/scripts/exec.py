#!/usr/bin/env python3
#coding: utf-8

import sys
import SSH
import common
import config


if __name__ == '__main__':
    common.print_info('''how to use
    function: run command in remote machine
    example: ./exec.py id cmd1 cmd2 cmd3 ...
    ''')

    if not common.check_argv(sys.argv, 3):
        common.print_error('no enough arguments')
        exit(0)
    robot_id = sys.argv[1]

    ip_address = common.get_ip(config.conf_file, robot_id)
    if not common.check_net(ip_address):
        common.print_error('can not connect to host, please check network')
        exit(0)
    ssh_client = SSH.SSH(ip_address, config.username, config.password)
    shell = ssh_client.create_shell()
    for i in range(2, len(sys.argv)):
        shell.exec_command(sys.argv[i])