import os
import logging
import threading
import ftplib
from typing import Dict, Union

import subprocess as sub
from subprocess import DEVNULL, STDOUT



if __name__ == "__main__":
    logging.basicConfig(
        level=logging.DEBUG,
        format='[%(asctime)s] [%(levelname)s] [%(filename)s:%(lineno)s]: %(message)s',
        handlers=[
            logging.FileHandler('remote_bot_system.log'),
            logging.StreamHandler()
        ]
    )



class FtpMountPoint:
    
    __host: str
    __port: int
    __username: str
    __password: str
    __remote_dir: str
    __mount_point: str
    
    def __init__(
        self,
        host: str, 
        port: int, 
        username: str, 
        password: str, 
        remote_dir: str, 
        mount_point: str, 
    ) -> None:
        self.__host = host
        self.__port = port
        self.__username = username
        self.__password = password
        self.__remote_dir = remote_dir
        self.__mount_point = mount_point
        logging.info(f"FtpPoint initialized, host:{host}, port:{port}, username:{username}, remote_dir:{remote_dir}, mount_point:{mount_point}")
        
    @property
    def path(self):
        return self.__mount_point
    
    def __ftp_service_available(self) -> bool:
        try:
            with ftplib.FTP(self.__host) as ftp:
                ftp.login(user=self.__username, passwd=self.__password)
                ftp.cwd('/')
                logging.debug("ftp service is available now")
                return True
        except Exception as e:
            logging.debug(f"ftp service is unavailable now, error info :{e}")
            return False
        
    def __exec_sys_cmd(self, cmd, time_wait = 2, stdout=DEVNULL, stderr=STDOUT) -> int:
        logging.debug(f"execute system cmd by sub.Popen, cmd: [{cmd}]")
        try:
            sub_process = sub.Popen(cmd, shell=True, stdout=stdout, stderr=stderr)
            sub_process.wait(time_wait)
        except Exception as e:
            logging.warning(e)
            return 1
        err = sub_process.poll()
        if err:
            logging.debug(f"failed to execute system cmd: {cmd}")
        return err
        
    def mount(self) -> int:
        if not os.path.exists(self.__mount_point):
            os.makedirs(self.__mount_point)
        if self.__ftp_service_available():
            logging.info(f"mount ftp folder, host:{self.__host} port:{self.__port} username:{self.__username} remote_dir:{self.__remote_dir} mount_point:{self.__mount_point}")
            cmd = f"curlftpfs -o rw,allow_other ftp://{self.__username}:{self.__password}@{self.__host}:{self.__port}{self.__remote_dir} {self.__mount_point}"
            err = self.__exec_sys_cmd(cmd)
            return err
        else:
            logging.info("ftp service unavailable, do not execute mount ftp folder action")
            return 1
        
    def unmount(self) -> int:
        if self.__ftp_service_available():
            logging.info(f"unmount ftp folder, host:{self.__host} port:{self.__port} username:{self.__username} remote_dir:{self.__remote_dir} mount_point:{self.__mount_point}")
            cmd = f"fusermount -u {self.__mount_point}"
            err = self.__exec_sys_cmd(cmd)
            return err
        else:
            logging.info("ftp service unavailable, do not execute unmount ftp folder action")
    
    
    

class FtpMountPointManager:
    
    __mount_point_dict: Dict[str, FtpMountPoint]
    __lock: threading.RLock
    __mount_point_root: str
    
    def __init__(self, mount_point_root:str) -> None:
        self.__mount_point_root = mount_point_root
        self.__clear_mount_point_root()
        self.__mount_point_dict = {}
        self.__lock = threading.RLock()
        
    def print(self, message:str):
        try:
            logging.debug(message)
        except:
            print(message)
        
    def __clear_mount_point_root(self):
        for robot_uuid in os.listdir(self.__mount_point_root):
            robot_mount_point_folder = os.path.join(self.__mount_point_root, robot_uuid)
            for robot_username in os.listdir(robot_mount_point_folder):
                mount_point = os.path.join(robot_mount_point_folder, robot_username)
                self.print(f"unmount: {mount_point}")
                cmd = f"fusermount -u {mount_point}"
                try:
                    sub_process = sub.Popen(cmd, shell=True, stdout=DEVNULL, stderr=STDOUT)
                    sub_process.wait(2)
                except Exception as e:
                    self.print(f"unmount failed: {e}")
                    pass
                err = sub_process.poll()
            
    def get(self, mount_point: str) -> Union[FtpMountPoint, None]:
        with self.__lock:
            if mount_point in self.__mount_point_dict:
                return self.__mount_point_dict[mount_point]
            else:
                return None
        
    def create(
        self, 
        host: str, 
        port: int, 
        username: str, 
        password: str, 
        remote_dir: str, 
        mount_point: str
    ) -> Union[FtpMountPoint, None]:
        if self.get(mount_point) is not None:
            logging.debug(f"ftp mount point [{mount_point}] already mounted, skip create")
            return self.get(mount_point)
        else:
            mount_point_entity = FtpMountPoint(host, port, username, password, remote_dir, mount_point)
            err = mount_point_entity.mount()
            if err != 0:
                return None
            with self.__lock:
                self.__mount_point_dict[mount_point] = mount_point_entity
            return mount_point_entity
    
    def release(self, mount_point:str) -> None:
        mount_point_entity = self.get(mount_point)
        if mount_point_entity is not None:
            mount_point_entity.unmount()
            with self.__lock:
                del self.__mount_point_dict[mount_point]
            logging.info(f"mount point [{mount_point}] released")
        else:
            logging.debug(f"mount point [{mount_point}] not exsist, terminate release process")
            
    @classmethod
    def get_mount_point_path(self, robot_uuid:str, robot_user_name:str):
        return f"/mnt/ftp/{robot_uuid}/{robot_user_name}"
            
    def __del__(self):
        self.__clear_mount_point_root()
            




ftp_mount_point_manager:FtpMountPointManager = FtpMountPointManager("/mnt/ftp")




if __name__ == "__main__":
    mmount_point_root = "/mnt/ftp"
    ftp_host = "192.168.124.139"
    ftp_port = 21
    ftp_username = "setsuna"
    ftp_password = "1"
    remote_dir = "/"
    mmount_point = "/mnt/ftp/mock_bot_workspace"
    
    import time
    ftp_mount_point_manager.create(ftp_host, ftp_port, ftp_username, ftp_password, remote_dir, mmount_point)
    time.sleep(5)
    ftp_mount_point_manager.release(mmount_point)
    while True:
        time.sleep(100)