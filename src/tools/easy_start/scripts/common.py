#!/usr/bin/env python3
#coding: utf-8

import subprocess
import json
import config
import os
import hashlib


def print_error(err):
    print('\033[1;31m %s \033[0m'%err)


def print_info(info):
    print('\033[1;32m %s \033[0m'%info)


def get_json_from_conf(confname=''):
    json_data = ''
    for line in open(confname): 
        count_of_quotatuion = 0
        for c in line:
            if c == '\'' or c == '\"':
                count_of_quotatuion += 1
            if c == '#' and count_of_quotatuion%2 == 0:
                break
            json_data += c
    return json_data


def run_cmd(cmd, prt=True):
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = p.communicate()
    p.wait()
    if p.poll() == 0:
        return True, out.decode('utf-8'), err.decode('utf-8')
    else:
        return False, out.decode('utf-8'), err.decode('utf-8')
        

def get_ip(cfg, id):
    conf = json.loads(get_json_from_conf(cfg))
    try:
        players = conf.get('players')
        player = players.get(id)
        return player.get('address')
    except:
        return None


def check_net(ip):
    backinfo = os.system('ping -c 1 -w 1 %s'%ip)
    if not backinfo:
        return True
    else:
        return False
    
    
def get_config(fname, key=''):
    conf = json.loads(get_json_from_conf(fname))
    keys = key.split('.')
    try:
        for k in keys:
            conf = conf.get(k)
        return conf
    except:
        return None


def parse_argv(argv=[], start=2):
    args = ' -p'+argv[1]
    for i in range(start,len(argv)):
        if '-j' in argv[i]:
            continue
        args += (' '+argv[i])
    return args


def check_argv(argv=[], num=2):
    if len(argv) < num:
        return False
    else:
        return True
