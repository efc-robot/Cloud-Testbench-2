if __name__ == "__main__": # for test
    import sys 
    sys.path.append("..")

import logging
import threading
import copy
from typing import Dict, Union

from models.robot import RobotInfo
from models.user import UserInfo
from models.ftp import FtpInfo

from features.robot import Robot, robot_manager




if __name__ == "__main__":
    logging.basicConfig(
        level=logging.DEBUG,
        format='[%(asctime)s] [%(levelname)s] [%(filename)s:%(lineno)s]: %(message)s',
        handlers=[
            logging.FileHandler('remote_bot_system.log'),
            logging.StreamHandler()
        ]
    )




class User:
    
    __info: UserInfo
    __robots: Dict[str, Robot]
    __lock: threading.RLock
    
    def __init__(self, user_info:UserInfo) -> None:
        self.__info = user_info
        self.__robots = {}
        self.__lock = threading.RLock()
        logging.info(f"user entity initialized, user uuid: [{user_info.uuid}]")
        
    @property
    def uuid(self) -> str:
        return self.__info.uuid.__str__()
        
    def get_my_robot(self, robot_uuid:str, robot_username:str) -> Union[Robot, None]:
        entity_id = robot_manager.get_entity_id(robot_uuid, robot_username)
        with self.__lock:
            if entity_id in self.__robots:
                return self.__robots[entity_id]
            else:
                return None
            
    def get_all_my_robots(self):
        return self.__robots.keys()
        
    def acquire_robot(self, robot_uuid:str, robot_username:str, password:str, workspace:str) -> bool:
        entity_id = robot_manager.get_entity_id(robot_uuid, robot_username)
        if self.get_my_robot(robot_uuid, robot_username) is not None:
            logging.debug(f"user [{self.__info.uuid}] already own robot [{entity_id}], terminate robot acquire process")
            return True
        else:
            success = robot_manager.allocate(robot_uuid, robot_username, self.__info.uuid.__str__(), password, workspace)
            if success:
                new_robot = robot_manager.get(robot_uuid, robot_username)
                with self.__lock:
                    self.__robots[new_robot.entity_id] = new_robot
                logging.info(f"user [{self.__info.uuid}] acquire robot success, get robot [{new_robot.entity_id}]")
                return True
            else:
                logging.info(f"user [{self.__info.uuid}] acquire robot failed")
                return False
            
    def release_robot(self, robot_uuid:str, robot_username:str) -> bool:
        entity_id = robot_manager.get_entity_id(robot_uuid, robot_username)
        if self.get_my_robot(robot_uuid, robot_username) is None:
            logging.debug(f"user [{self.__info.uuid}] not own robot [{entity_id}], terminate robot release process")
            return True
        else:
            success = robot_manager.deallocate(robot_uuid, robot_username)
            if success:
                with self.__lock:
                    del self.__robots[entity_id]
                logging.info(f"user [{self.__info.uuid}] release robot [{entity_id}] success")
                return True
            else:
                logging.info(f"user [{self.__info.uuid}] release robot [{entity_id}] failed")
                return False
            
    def release_all_robot(self) -> None:
        with self.__lock:
            for entity_id in self.__robots:
                robot:Robot = self.__robots[entity_id]
                robot_uuid = robot.uuid.__str__()
                robot_username = robot.robot_username
                robot_manager.deallocate(robot_uuid, robot_username)
            self.__robots = {}
            
    def exit(self) -> None:
        self.release_all_robot()
            
    
    
    

class UserManager:
    
    __lock: threading.RLock
    __users: Dict[str, User]
    __instance = None
    
    def __new__(cls):
        if not cls.__instance:
            cls.__instance = super().__new__(cls)
        return cls.__instance
    
    def __init__(self) -> None:
        self.__users = {}
        self.__lock = threading.RLock()
        logging.info("user manager initialized")
        
    def get(self, user_uuid:str) -> Union[User, None]:
        with self.__lock:
            if not user_uuid in self.__users:
                logging.debug(f"get user [{user_uuid}] failed, user entity not created")
                return None
            else:
                logging.debug(f"get user [{user_uuid}] success")
                return self.__users[user_uuid]
        
    def create(self, user_info:UserInfo) -> Union[User, None]:
        user_uuid = user_info.uuid.__str__()
        user = self.get(user_uuid)
        if user is not None:
            logging.debug(f"user [{user_uuid}] already created, terminate user create process")
            return user
        else:
            try:
                new_user = User(user_info)
                with self.__lock:
                    self.__users[user_uuid] = new_user
                logging.info(f"create user [{user_uuid}] success")
                return new_user
            except Exception as e:
                logging.warning(f"create user [{user_uuid}] failed: {e}")
                return None
            
    def remove(self, user_uuid) -> bool:
        user = self.get(user_uuid)
        if user is None:
            logging.debug(f"user [{user_uuid}] already removed, terminate user remove process")
            return True
        else:
            user.exit()
            with self.__lock:
                del self.__users[user_uuid]
            logging.info(f"remove user [{user_uuid}] success")
            return True




user_manager:UserManager = UserManager()
    
    
    
    
if __name__ == "__main__": # for test
    import time
    
    # create robot
    robot_info = RobotInfo()
    robot_info.id = 1
    robot_info.uuid = "9d944006-bcc2-11ed-b9c9-bfe419b26b81"
    robot_info.name = "mock bot 002"
    robot_info.type = "mock bot"
    robot_info.ip = "192.168.124.139"
    
    ftp_info = FtpInfo()
    ftp_info.id = 1
    ftp_info.robot_uuid = "9d944006-bcc2-11ed-b9c9-bfe419b26b81"
    ftp_info.username = "setsuna"
    ftp_info.password = "1"
    ftp_info.remote_dir = "/"
    ftp_info.mount_point = "/mnt/ftp/mock_bot_workspace"
    ftp_info.port = 21
    
    robot_manager.create(robot_info, ftp_info)
    
    # create user
    user_info = UserInfo()
    user_info.id = 1
    user_info.uuid = "8a940796-ac48-11ed-bbda-8565430a6f0c"
    user_info.username = "setsuna"
    user_info.password = "GUNDAM00"
    user_info.role = "user"
    
    # user = User(user_info)
    user = user_manager.create(user_info)
    
    # allocate robot
    robot_uuid = robot_info.uuid
    robot_username = ftp_info.username
    password = "1"
    workspace = "/home/coder/project"
    success = user.acquire_robot(robot_uuid, robot_username, password, workspace)
    
    time.sleep(5)
    
    if success:
        user_manager.remove(user.uuid)
        # user.release_robot(robot_uuid, robot_username)
        # user.release_all_robot()
        robot_manager.destory(robot_info.uuid, ftp_info.username)
        
    while True:
        time.sleep(100)