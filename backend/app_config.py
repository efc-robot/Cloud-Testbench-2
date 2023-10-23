import os
from pydantic import BaseModel


class Settings(BaseModel):
    # Base dir
    config_dir:str = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config")
    # Harbor
    harbor_host:str = "192.168.124.143"
    harbor_project:str = "cloud-testbench"


base_settings = Settings()