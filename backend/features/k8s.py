import os
from kubernetes import client, config
from kubernetes.client.api.core_v1_api import CoreV1Api
from typing import Dict, Union, List
from kubernetes.client.models.v1_pod import V1Pod
from kubernetes.client.models.v1_service import V1Service
from app_config import base_settings


# TODO: upgrade prints to logging
class K8sSimpleDeploy:
    
    __namespace: str
    __pod_name: str
    __nodePort_service_name: str
    __ports_mapping: Dict[str, int]
    
    __k8s_api: CoreV1Api
    
    def __init__(self) -> None:
        config_file = os.path.join(base_settings.config_dir, "kube_config")
        config.load_kube_config(config_file=config_file)
        self.__k8s_api:CoreV1Api = client.CoreV1Api()
        
    def __attach_pod_label_using_pod_name(self, pod:V1Pod) -> None:
        try:
            current_labels = pod.metadata.labels or {}
            current_labels["pod_name"] = self.__pod_name
            self.__k8s_api.patch_namespaced_pod(
                name=self.__pod_name,
                namespace=self.__namespace,
                body={"metadata": {"labels": current_labels}}
            )
        except Exception as e:
            print(e)
    
    def __create_pod(self, pod_manifest) -> bool:
        try:
            pod:V1Pod = self.__k8s_api.create_namespaced_pod(namespace=self.__namespace, body=pod_manifest)
            self.__pod_name = pod.metadata.name
            self.__attach_pod_label_using_pod_name(pod)
            print(f"new pod created, pod name [{self.__pod_name}]")
            return True
        except Exception as e:
            print(f"create pod failed: {e}")
            return False
            
    def __gennerate_nodePort_service_manifest(self, expose_ports):
        service_manifest = {
            "apiVersion": "v1",
            "kind": "Service",
            "metadata": {"generateName": f"{self.__pod_name}-nodeport-service-"},
            "spec": {
                "type": "NodePort",
                "selector": {"pod_name": self.__pod_name},
                "ports": [{"name": f"{port}", "protocol": "TCP", "port": port, "targetPort": port} for port in expose_ports],
            }
        }
        return service_manifest
    
    def __create_nodePort_service(self, service_manifest) -> bool:
        try:
            service:V1Service = self.__k8s_api.create_namespaced_service(body=service_manifest, namespace=self.__namespace)
            self.__ports_mapping = service.spec.ports
            self.__nodePort_service_name = service.metadata.name
            print(f"new nodePort service created, service name [{self.__nodePort_service_name}], port mapping: {self.__ports_mapping}")
            return True
        except Exception as e:
            print(f"create nodePort service failed: {e}")
            return False
    
    def __delete_pod(self) -> None:
        try:
            self.__k8s_api.delete_namespaced_pod(name=self.__pod_name, namespace=self.__namespace)
            print(f"pod [{self.__pod_name}] deleted")
        except Exception as e:
            print(f"delete pod [{self.__pod_name}] failed:{e}")
    
    def __delete_service(self) -> None:
        try:
            self.__k8s_api.delete_namespaced_service(name=self.__nodePort_service_name, namespace=self.__namespace)
            print(f"nodePort service [{self.__nodePort_service_name}] deleted")
        except Exception as e:
            print(f"delete nodePort service [{self.__nodePort_service_name}] failed:{e}")
    
    def init(self, namespace, pod_manifest, expose_ports:List[int]) -> bool:
        self.__namespace = namespace
        if not self.__create_pod(pod_manifest):
            print("create K8sSimpleDeployment failed at create pod")
            return False
        elif not self.__create_nodePort_service(self.__gennerate_nodePort_service_manifest(expose_ports)):
            self.__delete_pod()
            print("create K8sSimpleDeployment failed at create nodePort service")
            return False
        else:
            print(f"create K8sSimpleDeployment success, podName: {self.__pod_name}, serviceName: {self.__nodePort_service_name}")
            return True
        
    def destory(self):
        self.__delete_pod()
        self.__delete_service()
        print(f"K8sSimpleDeployment destoried, podName: {self.__pod_name}, serviceName: {self.__nodePort_service_name}")
    
    @property
    def pod_name(self) -> str:
        return self.__pod_name
    
    @property
    def namespace(self) -> str:
        return self.__namespace
    
    @property
    def nodePort_service_name(self) -> str:
        return self.__nodePort_service_name
    
    @property
    def ports_mapping(self) -> Dict[str, int]:
        return self.__ports_mapping
    
    
def create_k8sSimpleDeploy(
        name:str, container_image:str, 
        expose_ports:List[int]=[], container_env:Dict[str, str]={}, command:str=None, 
        nfs_ip:str=None, nfs_shared_folder:str=None, nfs_mount_path:str=None
    ) -> Union[K8sSimpleDeploy, None]:
    namespace = "default"
    name = name.replace("_", "-")
    # generate basic pod manifest
    pod_manifest = {
        "apiVersion": "v1",
        "kind": "Pod",
        "metadata": {"generateName": f"{name}-pod-"},
        "spec": {
            "containers": [
                {
                    "name": name, 
                    "image": f"{base_settings.harbor_host}/{base_settings.harbor_project}/{container_image}",
                    "ports": [{"containerPort": port} for port in expose_ports],
                    "env": [{"name": key, "value": value} for key, value in container_env.items()],
                }
            ],
            "imagePullSecrets": [{"name": "harbor-secret"}]
        }
    }
    # config nfs mount
    if (nfs_ip is None) or (nfs_shared_folder is None) or (nfs_mount_path is None):
        pass
    else:
        volume_name = "nfs-volume"
        pod_manifest["spec"]["volumes"] = [
            {
                "name": volume_name,
                "nfs": {
                    "server": nfs_ip, 
                    "path": nfs_shared_folder,
                }
            }
        ]
        for container_config in pod_manifest["spec"]["containers"]:
            container_config["volumeMounts"] = [{"name": volume_name, "mountPath": nfs_mount_path}]
    # config command
    if command is not None:
        for container_config in pod_manifest["spec"]["containers"]:
            container_config["command"] = command.split(" ")
    # create deploy
    K8s_deploy = K8sSimpleDeploy()
    success = K8s_deploy.init(namespace, pod_manifest, expose_ports)
    # return
    if not success:
        return None
    else:
        return K8s_deploy
    
    
if __name__ == "__main__":
    container_env = {
        "PASSWORD": "1", 
        "SUDO_PASSWORD": "1", 
        "DEFAULT_WORKSPACE": "/home", 
    }
    create_k8sSimpleDeploy(
        name="visual-robot",
        container_image="lscr.io/linuxserver/code-server:latest",
        expose_ports=[8080, 9090, 8443],
        container_env=container_env,
        nfs_ip="192.168.124.147",
        nfs_shared_folder="/export/shared",
        nfs_mount_path="/home"
    )