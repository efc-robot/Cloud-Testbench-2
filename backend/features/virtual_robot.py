import logging
import threading
from typing import Dict, List, Union, Optional
# import socket
# import docker
# from docker.models.containers import Container
# from docker.models.networks import Network
# from docker.client import DockerClient
# from features.port_allocator import port_allocator
from features.k8s import create_k8sSimpleDeploy, K8sSimpleDeploy
from pydantic import BaseModel


class VirtualBot:
    
    __name: str
    __password: str
    __sudo_password: str
    __default_workspace: str
    __ports: Dict[str, int]
    __docker_image: str
    __container_id: str
    
    __k8s_deploy: K8sSimpleDeploy
    
    def __init__(
        self, 
        name:str,
        password:Union[str, None] = None,
        sudo_password:Union[str, None] = None,
        default_workspace:str = None,
        expose_ports: List[int] = [],
        docker_image:str = "ros:cloud_test_bench", 
        entrypoint:str = None
    ) -> None:
        logging.debug(f"start to create virtual robot[name:{name}, default_workspace:{default_workspace}]")
        try:
            # # set environment
            # environment = []
            # environment.append(F"SERVICE_PORT={port_allocator.get()}")
            # if password is not None:
            #     environment.append(f"PASSWORD={password}")
            # if sudo_password is not None:
            #     environment.append(f"SUDO_PASSWORD={sudo_password}")
            # if default_workspace is not None:
            #     environment.append(f"DEFAULT_WORKSPACE={default_workspace}")
            # # set expose ports
            # ports = {}
            # if not 8443 in expose_ports:
            #     expose_ports.append(8443)
            # if not 9091 in expose_ports:
            #     expose_ports.append(9091)
            # host_ports = port_allocator.get_free_ports(len(expose_ports))
            # for i,  expose_port in enumerate(expose_ports):
            #     ports[f"{expose_port}"] = host_ports[i]
            # # create container
            # docker_client:DockerClient = docker.from_env()
            # new_container:Container = docker_client.containers.run(
            #     image = docker_image, 
            #     environment = environment,
            #     ports = ports, 
            #     privileged = True, 
            #     detach = True,
            #     network_mode = "bridge"
            # )
            # # connect to macvlan network
            # macvlan_network:Network = docker_client.networks.get('macvlan')
            # macvlan_network.connect(new_container)
            # # set prooerty
            # self.__name = name
            # self.__password = password
            # self.__sudo_password = sudo_password
            # self.__default_workspace = default_workspace
            # self.__ports = ports
            # self.__container_id = new_container.id
            # self.__docker_image = docker_image
            # # close docker client
            # docker_client.close()
            # logging.debug(f"create new virtual robot success, id:{self.__container_id}")
            
            # set environment
            environment = {}
            if password is not None:
                environment["PASSWORD"] = password
            if password is not None:
                environment["SUDO_PASSWORD"] = sudo_password
            if password is not None:
                environment["DEFAULT_WORKSPACE"] = default_workspace
            # create k8s deploy
            k8s_deploy_name = f"visual-bot-{name}"
            self.__k8s_deploy = create_k8sSimpleDeploy(k8s_deploy_name, docker_image, expose_ports, environment, command)
            if self.__k8s_deploy is None:
                raise Exception("create k8s deployment failed")
            else:
                # record ports mapping
                self.__ports = {}
                for mapping_info in self.__k8s_deploy.ports_mapping:
                    container_port = mapping_info.target_port
                    node_port = mapping_info.node_port
                    self.__ports[f"{container_port}"] = node_port
                # set prooerty
                self.__name = name
                self.__password = password
                self.__sudo_password = sudo_password
                self.__default_workspace = default_workspace
                self.__container_id = self.__k8s_deploy.pod_name
                self.__docker_image = docker_image
                logging.debug(f"create new virtual robot success, id:{self.__container_id}")
        except Exception as e:
            logging.debug(f"create new virtual robot failed: {e}")
        
    @property
    def name(self) -> str:
        return self.__name
        
    @property
    def password(self) -> str:
        return self.__password
        
    @property
    def sudo_password(self) -> str:
        return self.__sudo_password
        
    @property
    def default_workspace(self) -> str:
        return self.__default_workspace
        
    @property
    def id(self) -> str:
        return self.__container_id
        
    @property
    def ports(self) -> str:
        return self.__ports
        
    @property
    def docker_image(self) -> str:
        return self.__docker_image
    
    def destory(self) -> None:
        # docker_client:DockerClient = docker.from_env()
        # # disconnect container from macvlan network
        # container = docker_client.containers.get(self.__container_id)
        # macvlan_network:Network = docker_client.networks.get('macvlan')
        # macvlan_network.disconnect(container)
        # # remove container
        # docker_client.api.remove_container(container=self.__container_id, force=True)
        # # close client
        # docker_client.close()
        self.__k8s_deploy.destory()
        
        
class VirtualBotRecord(BaseModel):
    owner: Optional[str]
    id:str
    name: Optional[str]
    password: Optional[str]
    sudo_password: Optional[str]
    expose_ports: Optional[Dict[str, int]]
    default_workspace: Optional[str]
    docker_image: str
    

class VirtualBotManager:
    
    __lock: threading.RLock
    __virtual_bots: Dict[str, VirtualBot]
    __virtual_bots_info: Dict[str, VirtualBotRecord]
    
    def __init__(self) -> None:
        self.__lock = threading.RLock()
        self.__virtual_bots = {}
        self.__virtual_bots_info = {}
    
    def create(
        self,
        name:str,
        owner:str = None,
        password:Union[str, None] = None,
        sudo_password:Union[str, None] = None,
        default_workspace:str = "/",
        expose_ports: List[int] = [],
        docker_image:str = "ros:cloud_test_bench",
        command:str = None
    ) -> Union[VirtualBot, None]:
        logging.info(f"create new virtual robot")
        try:
            new_vBot:VirtualBot = VirtualBot(
                name=name, password=password, sudo_password=sudo_password,
                expose_ports=expose_ports, default_workspace=default_workspace,
                docker_image=docker_image, command=command
            )
            self.__register(new_vBot, owner)
            return new_vBot
        except Exception as e:
            logging.info(f"virtual robot manager create new virtual robot failed: {e}")
            return None
        
    def get(self, virtual_bot_id:str) -> Union[VirtualBot, None]:
        logging.debug(f"get virtual robot [{virtual_bot_id}]")
        with self.__lock:
            if not virtual_bot_id in self.__virtual_bots:
                logging.debug(f"virtual robot [{virtual_bot_id}] not exists")
                return None
            return self.__virtual_bots[virtual_bot_id]
                
    def destory(self, virtual_bot_id:str) -> bool:
        logging.debug(f"destory virtual robot [{virtual_bot_id}]")
        try:
            v_bot = self.get(virtual_bot_id)
            v_bot.destory()
            self.__unregister(virtual_bot_id)
            logging.debug(f"destory virtual robot [{virtual_bot_id}] success")
            return True
        except Exception as e:
            logging.debug(f"destory virtual robot [{virtual_bot_id}] failed: {e}")
            return False
            
    def get_visual_robot_info(self, virtual_bot_id:str) -> Union[VirtualBotRecord, None]:
        with self.__lock:
            try:
                return self.__virtual_bots_info[virtual_bot_id]
            except:
                return None
            
    def all_visual_robot_info(self) -> List[VirtualBotRecord]:
        return list(self.__virtual_bots_info.values())
    
    def filter_info_by_owner(self, owner:str=None) -> List[VirtualBotRecord]:
        if owner is None:
            return self.all_visual_robot_info()
        records = []
        with self.__lock:
            for id, record in self.__virtual_bots_info.items():
                if owner == record.owner:
                    records.append(record)
        return records
        
    def __register(self, vBot_entity:VirtualBot, owner:str=None) -> None:
        vBot_id = vBot_entity.id
        logging.debug(f"register new virtual robot [{vBot_id}]")
        new_record = VirtualBotRecord(
            owner=owner,
            id=vBot_id,
            name=vBot_entity.name,
            password=vBot_entity.password,
            sudo_password=vBot_entity.sudo_password,
            expose_ports=vBot_entity.ports,
            default_workspace=vBot_entity.default_workspace,
            docker_image=vBot_entity.docker_image
        )
        with self.__lock:
            self.__virtual_bots_info[vBot_id] = new_record
        with self.__lock:
            self.__virtual_bots[vBot_id] = vBot_entity
        logging.debug(f"new virtual robot [{vBot_id}] registed")
        
    def __unregister(self, vBot_id:str):
        logging.debug(f"unregister new virtual robot [{vBot_id}]")
        with self.__lock:
            del self.__virtual_bots_info[vBot_id]
        with self.__lock:
            del self.__virtual_bots[vBot_id]
        logging.debug(f"new virtual robot [{vBot_id}] unregisted")
            
        
virtual_bot_manager = VirtualBotManager()
        

if __name__ == "__main__":
    v_bot:VirtualBot = virtual_bot_manager.create(name="v_bot", owner="setsuna", expose_ports=[1211])
    import time
    time.sleep(5)
    print(virtual_bot_manager.filter_info_by_owner("admin"))
    print(virtual_bot_manager.filter_info_by_owner("setsuna"))
    print(virtual_bot_manager.get_visual_robot_info(v_bot.id))
    print(virtual_bot_manager.get_visual_robot_info(v_bot.id).__dict__)
    # virtual_bot_manager.destory(v_bot.id)