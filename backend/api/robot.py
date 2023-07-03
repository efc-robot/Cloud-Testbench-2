import uuid
import os
import socket
import netifaces as ni
from fastapi import APIRouter, HTTPException, Depends
from tortoise.contrib.fastapi import HTTPNotFoundError

from pydantic import BaseModel
from models.status import Status
from models.robot import RobotInfo_Pydantic, RobotInfoIn_Pydantic, RobotInfoIn_Pydantic_Post, RobotInfo
from models.ftp import FtpInfo

from features.token import verify_x_token
from features.robot import robot_manager
from features.user import user_manager, User
from features.ftp import ftp_mount_point_manager


router = APIRouter(tags=["robot"])


@router.get(
    "/robot/", 
    response_model=RobotInfo_Pydantic, 
    responses={404: {"model": HTTPNotFoundError}},
    description="get robot info by uuid"
)
async def get_robot_info(uuid: str):
    return await RobotInfo_Pydantic.from_queryset_single(RobotInfo.get(uuid=uuid))


@router.patch(
    "/robot/", 
    response_model=RobotInfo_Pydantic, 
    responses={404: {"model": HTTPNotFoundError}}
)
async def update_robot_info(robot_info: RobotInfoIn_Pydantic):
    await RobotInfo.filter(uuid=robot_info.uuid).update(**robot_info.dict(exclude_unset=True))
    return await RobotInfo_Pydantic.from_queryset_single(RobotInfo.get(uuid=robot_info.uuid))


@router.post("/robot/" , response_model=RobotInfoIn_Pydantic)
async def create_robot_info(robot_info: RobotInfoIn_Pydantic_Post):
    robot_info_dict = robot_info.dict(exclude_unset=True)
    robot_info_dict["uuid"] = uuid.uuid1()
    robot_info_obj = await RobotInfo.create(**robot_info_dict)
    return await RobotInfo_Pydantic.from_tortoise_orm(robot_info_obj)

    
@router.delete("/robot/", response_model=Status, responses={404: {"model": HTTPNotFoundError}})
async def delete_robot_info(uuid: str):
    deleted_count = await RobotInfo.filter(uuid=uuid).delete()
    if not deleted_count:
        raise HTTPException(status_code=404, detail=f"robot not found: uuid={uuid}")
    return Status(success=True, message=f"Deleted robot, uuid={uuid}")
    
    
class Form_RobotIdentify(BaseModel):
    robot_uuid: str
    robot_username: str

@router.post("/robot/online" , response_model=Status)
async def robot_online(data: Form_RobotIdentify):
    robot_info = await RobotInfo.get_or_none(uuid=data.robot_uuid)
    ftp_info = await FtpInfo.get_or_none(robot_uuid=data.robot_uuid, username=data.robot_username)
    
    ftp_mount_point = ftp_mount_point_manager.get_mount_point_path(data.robot_uuid, data.robot_username)
    if not os.path.exists(ftp_mount_point):
        os.makedirs(ftp_mount_point)
    if robot_info is None:
        return Status(success=False, message="robot sign in system fialed, robot not regist in system")
    elif ftp_info is None:
        return Status(success=False, message="robot sign in system fialed, robot username not regist in system")
    else:
        robot_entity = robot_manager.get(data.robot_uuid, data.robot_username)
        if robot_entity is not None:
            robot_entity.update_heartbeat()
            return Status(success=True, message="robot sign in system already, update heartbeat")
        else:
            new_robot = robot_manager.create(robot_info, ftp_info)
            if new_robot is None:
                return Status(success=False, message="robot sign in system fialed, error happen durinig create")
            else:
                return Status(success=True, message="robot sign in system seccess")
        

@router.get(
    "/robot/all", 
    responses={404: {"model": HTTPNotFoundError}},
    description="get all robot info"
)
async def get_all_robot_info():
    all_robots_info = []
    online_robots = robot_manager.get_online_robots()
    allocated_robots = robot_manager.get_allocated_robots()
    for robot_info in await RobotInfo.all().values():
        robot_uuid = robot_info["uuid"].__str__()
        robot_info["online"] = robot_uuid in online_robots
        robot_info["allocated"] = robot_uuid in allocated_robots
        all_robots_info.append(robot_info)
    return all_robots_info

@router.get(
    "/robot/get_my_robots", 
    responses={404: {"model": HTTPNotFoundError}},
    description="get all robot info"
)
async def get_my_robots_info(token_payload: dict = Depends(verify_x_token)):
    user_uuid = token_payload["user_uuid"]
    user:User = user_manager.get(user_uuid)
    if user is None:
        raise HTTPException(status_code=404, detail=f"user not found: user_uuid={user_uuid}")
    else:
        entity_id_list = user.get_all_my_robots()
        robots_info = []
        for entity_id in entity_id_list:
            robot = robot_manager.get_by_entity_id(entity_id)
            if robot is None:
                continue
            robots_info.append(
                {
                    "robot_name": robot.name,
                    "robot_uuid": robot.uuid,
                    "robot_username": robot.robot_username,
                    "robot_password": robot.robot_password,
                    "robot_type": robot.type,
                    "robot_ip": robot.ip,
                    "robot_workspace": robot.robot_workspace,
                    "code_server_workspace": robot.code_server_workspace,
                    "code_server_port": robot.code_server_port,
                    "code_server_password": robot.code_server_password
                }
            )
        return robots_info

@router.post("/robot/release" , response_model=Status)
async def robot_release(data: Form_RobotIdentify, token_payload: dict = Depends(verify_x_token)):
    user_uuid = token_payload["user_uuid"]
    user = user_manager.get(user_uuid)
    if user is None:
        raise HTTPException(status_code=404, detail=f"user not found: user_uuid={user_uuid}")
    success = user.release_robot(data.robot_uuid, data.robot_username)
    if success:
        return Status(success=success, message="robot release success")
    else:
        return Status(success=success, message="robot release fialed")