import docker
import threading
import logging
import socket
from typing import Dict, Union
from docker.models.containers import Container
from features.port_allocator import port_allocator




if __name__ == "__main__":
    logging.basicConfig(
        level=logging.DEBUG,
        format='[%(asctime)s] [%(levelname)s] [%(filename)s:%(lineno)s]: %(message)s',
        handlers=[
            logging.FileHandler('remote_bot_system.log'),
            logging.StreamHandler()
        ]
    )




class CodeServerContainer(Container):
    service_port: int
    workspace: str
    password: str




class CodeServerContainerManager:
    
    __lock: threading.RLock
    __code_server_containers: Dict[str, CodeServerContainer]

    def __init__(self):
        self.__code_server_containers = {}
        self.__lock = threading.RLock()
        logging.info("code server container manager initialized")

    def create_container(
        self, 
        password: str, 
        host_dir: str,
        container_dir: str,
        port: int = None, 
        container_name: str = None
    ) -> Union[CodeServerContainer, None]:
        logging.info(f"create new code server container, mount host dir [{host_dir}] to [{container_dir}] inside the container.")
        try:
            docker_client = docker.from_env()
            if port is None:
                port = port_allocator.get()
            new_container:CodeServerContainer = docker_client.containers.run(
                name = container_name,
                image = "lscr.io/linuxserver/code-server:latest",
                volumes = [f"{host_dir}:{container_dir}"],
                environment = [
                    f"PASSWORD={password}",
                    f"SUDO_PASSWORD={password}",
                    f"DEFAULT_WORKSPACE={container_dir}"
                ],
                ports = {"8443/tcp": port},
                privileged = True,
                detach = True
            )
            docker_client.close()
            new_container.service_port = port
            new_container.workspace = container_dir
            new_container.password = password
            with self.__lock:
                self.__code_server_containers[new_container.id] = new_container
            return new_container
        except Exception as e:
            logging.warning(f"error when create code server container: {e}")
            return None

    def remove_container(self, container_id:str) -> bool:
        # remove container data in self.__code_server_containers
        with self.__lock:
            if not container_id in self.__code_server_containers:
                logging.debug(f"container manager can not find [container_id], terminate remove process")
            else:
                target_container:CodeServerContainer = self.__code_server_containers[container_id]
                del self.__code_server_containers[container_id]
        # remove docker container
        container_removed = False
        try:
            logging.info(f"remove code server container [container_id]")
            target_container.remove(force=True)
            container_removed = True
        except Exception as e:
            logging.warning(f"error when remove code server container: {e}")
        return container_removed
    
    def get(self, container_id:str) -> Union[CodeServerContainer, None]:
        with self.__lock:
            if container_id in self.__code_server_containers:
                return self.__code_server_containers[container_id]
            else:
                return None
            
    def __del__(self):
        for container_id in self.__code_server_containers:
            self.__code_server_containers[container_id].remove(force=True)
        



code_server_container_manager = CodeServerContainerManager()



if __name__ == "__main__":
    container_name = "test"
    port = 9080
    password = "1"
    host_dir = "/mnt/ftp/mock_bot_workspace"
    container_dir = "/home/coder/project"
    name = "test"
    
    container = code_server_container_manager.create_container(
        password,
        host_dir,
        container_dir,
        port,
        name
    )
    if container is not None:
        code_server_container_manager.remove_container(container.id)
        
    import time
    while True:
        time.sleep(100)