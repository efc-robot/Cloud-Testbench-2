# from typing import List, Union

# from fastapi import APIRouter
# from fastapi.encoders import jsonable_encoder
# from pydantic import BaseModel

# from models.status import Status
# from models.robot import RobotInfo
# from models.user import UserInfo
# from models.ftp import FtpInfo

# router = APIRouter(tags=["code server"])
    
    
# new_ws_node = None


# class Form_create_codeserver_container(BaseModel):
#     uuid_user: str
#     uuid_robot: str
#     ftp_username: str
#     code_server_password: str


# class Form_del_codeserver_container(BaseModel):
#     uuid_robot: str
#     ftp_username: str
    
    
# @router.post("/code_server/", response_model=Status)
# async def create_code_server_workspace(form_data: Form_create_codeserver_container):
#     user_info = await UserInfo.get(uuid=form_data.uuid_user)
#     robot_info = await RobotInfo.get(uuid=form_data.uuid_robot)
#     ftp_info = await FtpInfo.get(robot_uuid=form_data.uuid_robot, username=form_data.ftp_username)
    
#     from features.ftp import ftp_mount_point_manager
#     from features.vscode import code_server_container_manager
    
    
    
#     # from code_server.vscode import CodeWorkspaceNode
#     # global new_ws_node
#     # new_ws_node = CodeWorkspaceNode()
#     # new_ws_node.construct(
#     #    robot_info.ip,
#     #    robot_info.port,
#     #    ftp_info.username, 
#     #    ftp_info.password, 
#     #    ftp_info.remote_dir, 
#     #    ftp_info.mount_point, 
#     #    f"ctb_codeserver__CU_{user_info.uuid}__B_{robot_info.uuid}__FU_{ftp_info.username}", 
#     #    "/home/coder/project", 
#     #    9080, 
#     #    form_data.code_server_password
#     # )
    
#     from features.vscode import CodeServerContainer
#     global new_ws_node
#     new_ws_node = CodeServerContainer()
#     new_ws_node.construct(
#         f"ctb_codeserver__CU_{user_info.uuid}__B_{robot_info.uuid}__FU_{ftp_info.username}", 
#         9080, form_data.code_server_password, ftp_info.mount_point, "/home", 
#     )
    
    
#     return Status(message="127.0.0.1:9080")


# @router.delete("/code_server/", response_model=Status)
# async def delete_code_server_workspace(form_data: Form_del_codeserver_container):
#     # user_info = await UserInfo.get(uuid=form_data.uuid_user)
#     # robot_info = await RobotInfo.get(uuid=form_data.uuid_robot)
    
#     # from code_server.code_workspace import CodeWorkspaceNode
#     global new_ws_node
#     new_ws_node.destory()
    
#     return Status(message="success")