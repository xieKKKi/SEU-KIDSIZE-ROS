import paramiko
import socket
import threading
import datetime


class ReadThread(threading.Thread):
    def __init__(self, shell):
        super(ReadThread, self).__init__()
        self._shell = shell
        self._shell.settimeout(0.2)
        self.__running = threading.Event()
        self.__running.set()

    def terminate(self):
        self.__running.clear()

    def run(self):
        while self.__running.isSet():
            try:
                x = self._shell.recv(1)
                print(x.decode('utf-8', 'ignore'), end='')
            except socket.timeout:
                pass


class Shell:
    def __init__(self, chan):
        self._shell = chan
        self._shell.settimeout(0.2)

    def exec_command(self, cmd):
        ret = ""
        self._shell.send(cmd+'\n')
        x = "  "
        while len(x) > 0:
            try:
                x = self._shell.recv(1)
                print(x.decode('utf-8', 'ignore'), end='')
                ret = ret + x.decode('utf-8', 'ignore')
            except socket.timeout:
                ret = self._del_color(ret)
                return "socket timeout" if len(ret) == 0 else ret
        ret = self._del_color(ret)
        return ret

    def _del_color(self, s):
        ret = ''
        l = len(s)
        i = 0
        while i < l :
            c = s[i]
            if c == '':
                while s[i] != 'm':
                    i = i+1
            else:
                ret = ret + c
            i = i + 1
        return ret
                

class SSH:
    def __init__(self, hostname, username, password):
        self._client = paramiko.SSHClient()
        self._client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.connected = False
        try:
            print("\rconnecting...")
            self._client.connect(hostname, username=username, password=password, timeout=5.0)
            self.connected = True
            self._sftp = self._client.open_sftp()
        except Exception as e:
            raise Exception("connect to {}@{} failed: {}".format(username, hostname, e))

    def close(self):
        self._client.close()

    def create_shell(self):
        if not self.connected:
            return None
        return Shell(self._client.invoke_shell())
    
    def transport_status(self, trans, total):
        print("\rtransport: {:.2f}%".format(float(trans)/total*100), end='')

    def upload(self, local, remote, callback=None):
        if not self.connected:
            return
        print('upload file from [{}] to [{}] \n'.format(local, remote))
        status = self.transport_status if callback is None else callback
        self._sftp.put(local, remote, status)
        print('\nupload file complete')

    def download(self, remote, local, callback=None):
        if not self.connected:
            return
        print('download file from [{}] to [{}] \n'.format(remote, local))
        status = self.transport_status if callback is None else callback
        self._sftp.get(remote, local, status)
        print('\ndownload file complete')

    def exec_command(self, cmdlist):
        shell = self._client.invoke_shell()
        for cmd in cmdlist:
            shell.send(cmd + '\n')
        recv_t = ReadThread(shell)
        recv_t.start()
        return recv_t

    def exec_for_result(self, cmd):
        stdin, stdout, stderr = self._client.exec_command(cmd)
        stdin.close()
        return stdout.readlines()

    def get_remote_time(self):
        cmd = "date '+%Y-%m-%d %H:%M:%S'"
        return self.exec_for_result(cmd)[0]

    def set_remote_time(self):
        cmd = "date -s '{}'".format(datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
        self._client.exec_command(cmd)


