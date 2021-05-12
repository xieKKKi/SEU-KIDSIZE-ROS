#!/usr/bin/env python3
#coding: utf-8

import os
import re

project_name = 'SEU-Kidsize'
username = 'root'
password = 'nvidia'
local_root = os.path.realpath(__file__).split(project_name)[0]+project_name
conf_file = '{}/src/config/conf/config.conf'.format(local_root)
remote_root = '/home/nvidia/{}'.format(project_name)
start_up_file = '/etc/rc.local'
wan_file = '/etc/NetworkManager/system-connections/robocup'
lan_file = '/etc/NetworkManager/system-connections/static'
md5_file = 'md5.txt'
