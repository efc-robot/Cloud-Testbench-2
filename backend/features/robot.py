if __name__ == "__main__":
    import sys 
    sys.path.append("..")

import logging
import threading
import datetime
import time
from typing import Dict, List, Union

from models.robot import RobotInfo
from models.ftp import FtpInfo

# from features.ftp import ftp_mount_point_manager, FtpMountPoint
# from features.vscode import code_server_container_manager, CodeServerContainer
from features.k8s import create_k8sSimpleDeploy, K8sSimpleDeploy




if __name__ == "__main__":
    logging.basicConfig(
        level=logging.DEBUG,
        format='[%(asctime)s] [%(levelname)s] [%(filename)s:%(lineno)s]: %(message)s',
        handlers=[
            logging.FileHandler('remote_bot_system.log'),
            logging.StreamHandler()
        ]
    )




class Robot:
    
    __robot_info: RobotInfo
    __ftp_info: FtpInfo
    __entity_id: str
    
    __user_uuid: str = None
    # __ftp_mount_point: FtpMountPoint = None
    # __code_server_container: CodeServerContainer = None
    
    __online: bool
    __heartbeat: datetime.datetime
    __heartbeat_timeout_sec: int
    __heartbeat_check_interval: int
    __destory_timeout: int
    
    __code_server_port: int
    __code_server_workspace: str
    __code_server_password: str
    __k8s_deploy: K8sSimpleDeploy = None
    
    __lock: threading.Lock
    
    def __init__(
        self, robot_info: RobotInfo, ftp_info:FtpInfo, 
        heartbeat_timeout_sec:int = 3,
        heartbeat_check_interval:int = 10,
        destory_timeout:int = 60    
    ) -> None:
        self.__online = True
        self.__heartbeat_timeout_sec = heartbeat_timeout_sec
        self.__heartbeat_check_interval = heartbeat_check_interval
        self.__destory_timeout = destory_timeout
        self.__heartbeat = datetime.datetime.now()
        self.__robot_info = robot_info
        self.__ftp_info = ftp_info
        self.__entity_id = RobotManager.get_entity_id(robot_info.uuid.__str__(), ftp_info.username)
        self.__lock = threading.Lock()
        # self.mount_workspace(ftp_info)
        self.__start_online_status_refresh()
        
    @property
    def user_uuid(self) -> str:
        return self.__user_uuid
    
    @property
    def entity_id(self) -> str:
        return self.__entity_id
    
    @property
    def name(self) -> str:
        return self.__robot_info.name
    
    @property
    def ip(self) -> str:
        return self.__robot_info.ip
    
    @property
    def robot_username(self) -> str:
        return self.__ftp_info.username
    
    @property
    def robot_password(self) -> str:
        return self.__ftp_info.password
    
    @property
    def robot_workspace(self) -> str:
        return self.__ftp_info.remote_dir
    
    @property
    def code_server_port(self) -> int:
        return self.__code_server_port
    
    @property
    def code_server_workspace(self) -> str:
        return self.__code_server_workspace
    
    @property
    def code_server_password(self) -> str:
        return self.__code_server_password
    
    @property
    def type(self) -> str:
        return self.__robot_info.type
    
    @property
    def uuid(self) -> str:
        return self.__robot_info.uuid.__str__()
    
    @property
    def robot_username(self) -> str:
        return self.__ftp_info.username
    
    @property
    def online(self) -> bool:
        return self.__online
        
    def set_user(self, user_uuid:str) -> None:
        self.__user_uuid = user_uuid
        
    def remove_user(self) -> None:
        self.__user_uuid = None
        
    # def __workspace_mounted(self):
    #     if self.__ftp_mount_point is None:
    #         return False
    #     elif ftp_mount_point_manager.get(self.__ftp_mount_point.path) is None:
    #         return False
    #     else:
    #         return True
    
    # def mount_workspace(self, ftp_info: FtpInfo) -> bool:
    #     if self.__workspace_mounted():
    #         logging.debug(f"robot entity [{self.__entity_id}] workspace already mounted, terminate mount process")
    #         return True
    #     else:
    #         self.__ftp_mount_point = ftp_mount_point_manager.create(
    #             host=self.__robot_info.ip,
    #             port=self.__ftp_info.port,
    #             username=self.__ftp_info.username,
    #             password=self.__ftp_info.password,
    #             remote_dir=self.__ftp_info.remote_dir,
    #             mount_point=self.__ftp_info.mount_point
    #         )
    #     if self.__workspace_mounted():
    #         logging.debug(f"robot entity [{self.__entity_id}] mount workspace success")
    #         return True
    #     else:
    #         logging.debug(f"robot entity [{self.__entity_id}] mount workspace failed")
    #         return False
    
    # def unmount_workspace(self) -> None:
    #     if self.__workspace_mounted():
    #         ftp_mount_point_manager.release(self.__ftp_mount_point.path)
    #         logging.info(f"robot entity [{self.__entity_id}] workspace unmounted")
    #     else:
    #         logging.debug(f"robot entity [{self.__entity_id}] workspace not mounted, terminate unmount process")
            
    # def code_server_container_created(self) -> bool:
    #     if self.__code_server_container is None:
    #         return False
    #     elif code_server_container_manager.get(self.__code_server_container.id) is None:
    #         return False
    #     else:
    #         return True
    
    # def create_code_server_container(self, password:str, workspace:str, port:int=None, name:str=None) -> bool:
    #     if self.code_server_container_created():
    #         logging.debug(f"robot entity [{self.__entity_id}] already created code server container, terminate create process")
    #         return True
    #     else:
    #         self.__code_server_container = code_server_container_manager.create_container(
    #             password=password,
    #             host_dir=self.__ftp_info.mount_point,
    #             container_dir=workspace,
    #             port=port,
    #             container_name=name
    #         )
    #         container_created =  self.code_server_container_created()
    #         if container_created:
    #             logging.debug(f"code server container created by robot entity [{self.__entity_id}], container id: [{self.__code_server_container.id}]")
    #         else:
    #             logging.debug(f"robot entity [{self.__entity_id}] create code server container failed")
    #         return container_created
    
    # def release_code_server_container(self) -> bool:
    #     if not self.code_server_container_created():
    #         logging.debug(f"robot entity [{self.__entity_id}] not hold code server container, terminate release process")
    #         return True
    #     else:
    #         container_id = self.__code_server_container.id
    #         container_removed = code_server_container_manager.remove_container(container_id)
    #         if container_removed:
    #             logging.debug(f"robot [{self.__entity_id}] remove code server container [{container_id}] success")
    #             self.__code_server_container = None
    #             return True
    #         else:
    #             logging.debug(f"robot [{self.__entity_id}] remove code server container [{container_id}] failed")
    #             return False
        
    # def shutdown(self):
    #     if self.code_server_container_created():
    #         self.release_code_server_container()
    #     if self.__workspace_mounted():
    #         self.unmount_workspace()
    #     logging.info(f"robot {self.__entity_id} shutdown")
            
    def update_heartbeat(self) -> None:
        logging.debug(f"robot [{self.__entity_id}] update heartbeat")
        with self.__lock:
            self.__heartbeat = datetime.datetime.now()
            
    def _refresh_online_status_thread(self):
        while True:
            current_time = datetime.datetime.now()
            with self.__lock:
                delta_time_sec = (current_time - self.__heartbeat).seconds
            if delta_time_sec > self.__destory_timeout:
                logging.debug(f"robot [{self.__entity_id}] not receive heartbeat for {self.__destory_timeout} sec, destory robot entity")
                robot_manager.destory(self.uuid, self.robot_username)
                break
<<<<<<< HEAD
            if delta_time_sec > self.__heartbeat_timeout_sec:
                online = False
            else:
                online = True
            if online and (not self.__online):
=======
            online_now = delta_time_sec < self.__heartbeat_timeout_sec
            if online_now == self.__online:
                pass
            elif online_now:
>>>>>>> e141c1c8cb30567bf4cad567ead5b657737f2937
                logging.info(f"robot {self.uuid} online")
            else:
                logging.info(f"robot {self.uuid} offline")
            with self.__lock:
                self.__online = online_now
            time.sleep(self.__heartbeat_check_interval)
            
    def __start_online_status_refresh(self):
        self.__online_status_thread = threading.Thread(target=self._refresh_online_status_thread)
        self.__online_status_thread.daemon = True
        self.__online_status_thread.start()
    
        
    def create_k8s_deploy(self, password, default_workspace, service_port=8443) -> bool:
        container_env = {
            "PASSWORD": password, 
            "SUDO_PASSWORD": password, 
            "DEFAULT_WORKSPACE": default_workspace, 
        }
        k8s_deploy_name = f"robot-{self.__robot_info.name}"
        k8s_deploy = create_k8sSimpleDeploy(
            k8s_deploy_name, "lscr.io/linuxserver/code-server:latest", 
            [service_port], container_env, 
            self.__robot_info.ip, self.__ftp_info.remote_dir, default_workspace
        )
        success = (k8s_deploy is not None)
        if success:
            self.__k8s_deploy = k8s_deploy
            self.__code_server_workspace = default_workspace
            self.__code_server_password = password
            for mapping_info in k8s_deploy.ports_mapping:
                if mapping_info.port == service_port:
                    self.__code_server_port = mapping_info.node_port
                    break
            logging.debug(f"code server k8s deploy created by robot entity [{self.__entity_id}], pod name: [{k8s_deploy.pod_name}]")
        else:
            logging.debug(f"robot entity [{self.__entity_id}] create code server k8s deploy failed")
        return success
    
    def code_server_container_created(self):
        created = (self.__k8s_deploy is not None)
        return created
    
    def remove_k8s_deploy(self) -> bool:
        if self.__k8s_deploy is None:
            logging.debug(f"robot entity [{self.__entity_id}] not hold code server k8s deploy, terminate release process")
            return True
        else:
            self.__k8s_deploy.destory()
            self.__k8s_deploy = None
            logging.debug(f"robot [{self.__entity_id}] remove code server k8s deploy [{self.__k8s_deploy.pod_name}] removed")
            return True
        
    def shutdown(self):
        if self.code_server_container_created():
            self.remove_k8s_deploy()
        logging.info(f"robot {self.__entity_id} shutdown")
    
    
    
    
class RobotManager:
    
    __lock: threading.RLock
    __robots: Dict[str, Robot]
    
    def __init__(self) -> None:
        self.__robots = {}
        self.__lock = threading.RLock()
        logging.info("robot manager initialized")
    
    @classmethod
    def get_entity_id(self, robot_uuid:str, robot_username:str):
        entity_id = f"{robot_uuid}_{robot_username}"
        return entity_id
    
    def __get(self, entity_id:str) -> Union[Robot, None]:
        with self.__lock:
            if entity_id in self.__robots:
                return self.__robots[entity_id]
            else:
                return None
        
    def get_by_entity_id(self, entity_id:str) -> Union[Robot, None]:
        return self.__get(entity_id)
        
    def get(self, robot_uuid:str, robot_username:str) -> Union[Robot, None]:
        entity_id = self.get_entity_id(robot_uuid, robot_username)
        return self.__get(entity_id)
    
    def get_online_robots(self) -> List[str]:
        online_robots = []
        for entity_id in self.__robots:
            robot = self.__get(entity_id)
            if robot.online:
                online_robots.append(robot.uuid)
        return online_robots
    
    def get_allocated_robots(self) -> List[str]:
        allocated_roots = []
        for entity_id in self.__robots:
            robot = self.__get(entity_id)
            if self.__is_bot_allocated(robot):
                allocated_roots.append(robot.uuid)
        return allocated_roots
        
    def create(self, robot_info:RobotInfo, ftp_info:FtpInfo) -> Robot:
        robot_uuid = robot_info.uuid.__str__()
        robot_username = ftp_info.username
        entity_id = self.get_entity_id(robot_uuid, robot_username)
        robot = self.__get(entity_id)
        if robot is not None:
            logging.debug(f"robot [{entity_id}] already exsist, terminate create process")
            return robot
        else:
            robot = Robot(robot_info, ftp_info)
            if robot is not None:
                logging.info(f"create robot [{entity_id}] success")
                with self.__lock:
                    self.__robots[entity_id] = robot
                return robot
            else:
                logging.info(f"create robot [{entity_id}] failed")
                return None
            
    async def update_robot_heartbeat(self, robot_uuid:str, robot_username:str) -> None:
        robot_entity:Robot = self.get(robot_uuid, robot_username)
        if robot_entity is None:
            return
        robot_entity.update_heartbeat()
            
    def destory(self, robot_uuid:str, robot_username:str) -> None:
        entity_id = self.get_entity_id(robot_uuid, robot_username)
        robot = self.__get(entity_id)
        # return if robot not exsist
        if robot is None:
            logging.debug(f"robot [{entity_id}] not created yet, terminate destory process")
            return
        # deallocate bot if allocated
        if self.__is_bot_allocated(robot):
            self.deallocate(robot_uuid, robot_username)
        # shutdown bot
        robot.shutdown()
        # unregister bot
        with self.__lock:
            del self.__robots[entity_id]
        logging.info(f"destory robot [{entity_id}] success")
            
    def __is_bot_allocated(self, robot:Robot) -> bool:
        if robot.user_uuid is not None:
            logging.debug(f"robot [{robot.entity_id}] was allocated, user [{robot.user_uuid}]")
            return True
        elif robot.code_server_container_created():
            logging.debug(f"robot [{robot.entity_id}] was allocated, code server container created")
            return True
        else:
            logging.debug(f"robot [{robot.entity_id}] not allocated")
            return False
    
    def allocate(self, robot_uuid:str, robot_username:str, user_uuid:str, password:str, workspace:str) -> bool:
        entity_id = self.get_entity_id(robot_uuid, robot_username)
        robot = self.__get(entity_id)
        if robot is None:
            logging.info(f"robot allocate failed, robot entity [{entity_id}] not exsist")
            return False
        elif self.__is_bot_allocated(robot):
            logging.info(f"robot allocate failed, robot entity [{entity_id}] already allocated")
            return False
        elif not robot.create_k8s_deploy(password, workspace):
            logging.info(f"robot allocate failed, failed at create k8s deploy")
            return False
        else:
            robot.set_user(user_uuid)
<<<<<<< HEAD
=======
            container_name = f"codeServer.usr_{user_uuid}.bot_{robot_uuid}.botUsr_{robot_username}.{datetime.datetime.now().timestamp()}"
            robot.create_code_server_container(password, workspace, name=container_name)
>>>>>>> e141c1c8cb30567bf4cad567ead5b657737f2937
            logging.info(f"robot allocate success, robot entity [{entity_id}] allocated to user [{user_uuid}]")
            return True
        
    def deallocate(self, robot_uuid:str, robot_username:str) -> bool:
        entity_id = self.get_entity_id(robot_uuid, robot_username)
        robot = self.__get(entity_id)
        if robot is None:
            logging.debug(f"robot entity [{entity_id}] not exsist, terminate robot deallocate process")
            return True
        elif not self.__is_bot_allocated(robot):
            logging.debug(f"robot entity [{entity_id}] not allocated, terminate robot deallocate process")
            return True
        else:
            code_server_released = robot.remove_k8s_deploy()
            if not code_server_released:
                logging.info(f"robot entity [{entity_id}] deallocate failed, can not release code server container")
                return False
            else:
                robot.remove_user()
                logging.info(f"robot entity [{entity_id}] deallocate success")
                return True




robot_manager:RobotManager = RobotManager()




if __name__ == "__main__":
    import time

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
    
    code_server_password = "1"
    code_server_workspace = "/home/coder/project"
    
    user_uuid = "8a940796-ac48-11ed-bbda-8565430a6f0c"
    robot_uuid = robot_info.uuid
    robot_username = ftp_info.username
    
    robot_manager.create(robot_info, ftp_info)
    robot_manager.allocate(
        robot_uuid, robot_username, user_uuid, 
        code_server_password, 
        code_server_workspace
    )
    time.sleep(10)
    robot_manager.deallocate(robot_uuid, robot_username)
    robot_manager.destory(robot_uuid, robot_username)
    
    while True:
        time.sleep(100)
    