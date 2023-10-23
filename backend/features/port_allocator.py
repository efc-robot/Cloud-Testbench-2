import logging
import threading
import socket
import time
from typing import List


class PortAllocator:
    
    __host: str
    __start_port: int
    __end_port: int
    __free_ports: List[int]
    __lock: threading.RLock
    __scan_thread: threading.Thread
    __is_scanning: bool
    
    def __init__(
        self, host:str="localhost", start_port:int=49200, 
        end_port:int=49500, scan_interval_sec:int=60    
    ) -> None:
        logging.debug("init port allocator")
        self.__host = host
        self.__start_port = start_port
        self.__end_port = end_port
        self.__lock = threading.RLock()
        self.__is_scanning = False
        self.__scan_thread = None
        self.__start_ports_scan(scan_interval_sec)
        
    def get_free_ports(self, port_num:int) -> List[int]:
        time_start = time.perf_counter()
        logging.debug(f"allocate free port on {self.__host}, acquire port num: {port_num}")
        ret_ports = []
        with self.__lock:
            if port_num > len(self.__free_ports):
                raise Exception("no enough free port")
            for port in self.__free_ports:
                if port_num <= 0:
                    break
                if self.__is_port_free(port):
                    ret_ports.append(port)
                    port_num -= 1
                self.__free_ports.remove(port)
        time_end = time.perf_counter()       
        time_delta_sec = time_end - time_start
        time_delta_ms = time_delta_sec * 1000 
        logging.debug(f"allocate free port success, ports {ret_ports} allocated, time cost:{time_delta_ms}ms")
        return ret_ports
    
    def get(self) -> int:
        with self.__lock:
            for port in self.__free_ports:
                if self.__is_port_free(port):
                    self.__free_ports.remove(port)
                    return port
        
        
    def __scan_ports(self) -> None:
        logging.debug("scan free ports")
        time_start = time.perf_counter()
        with self.__lock:
            self.__free_ports = []
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                for port in range(self.__start_port, self.__end_port + 1):
                    if sock.connect_ex((self.__host, port)) != 0:
                        self.__free_ports.append(port)
        time_end = time.perf_counter()
        time_delta_sec = time_end - time_start
        time_delta_ms = time_delta_sec * 1000
        logging.debug(f"scan free ports finished, time cost: {time_delta_ms}ms")
        
    def _scan_ports_thread(self, scan_interval_sec:int):
        while self.__is_scanning:
            self.__scan_ports()
            time.sleep(scan_interval_sec)
            
    def __start_ports_scan(self, scan_interval_sec:int):
        self.__is_scanning = True
        self.__scan_thread = threading.Thread(
            target=self._scan_ports_thread,
            kwargs=({"scan_interval_sec":scan_interval_sec})
        )
        self.__scan_thread.daemon = True
        self.__scan_thread.start()
            
    def __stop_ports_scan(self):
        self.__is_scanning = False
        if self.__scan_thread is not None:
            self.__scan_thread.join()
            
    def __is_port_free(self, port:int) -> bool:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            return sock.connect_ex((self.__host, port)) != 0
        
    def __del__(self):
        self.__stop_ports_scan()
                
                
# port_allocator = PortAllocator()
                
                
if __name__ == "__main__":
        while True:
            # port_allocator.get_free_ports(5)
            time.sleep(5)