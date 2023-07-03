import uuid
from typing import List

from fastapi import APIRouter, HTTPException
from tortoise.contrib.fastapi import HTTPNotFoundError

from models.status import Status
from models.ftp import FtpInfo_Pydantic, FtpInfoIn_Pydantic, FtpInfoIn_Pydantic_Get, FtpInfoIn_Pydantic_Del, FtpInfo

from features.ftp import ftp_mount_point_manager


router = APIRouter(tags=["ftp info"])


@router.get(
    "/ftp_info/", 
    response_model=List[FtpInfo_Pydantic], 
    responses={404: {"model": HTTPNotFoundError}},
    description="get ftp info by robot uuid & ftp user name"
)
# async def get_ftp_info(ftp_info: FtpInfoIn_Pydantic_Get):
#     return await FtpInfo_Pydantic.from_queryset(FtpInfo.filter(**ftp_info.dict(exclude_unset=True)))
async def get_ftp_info(robot_uuid:str=None, username:str=None):
    q_dict = {}
    if robot_uuid is None and username is None:
        return await FtpInfo_Pydantic.from_queryset(FtpInfo.all())
    if robot_uuid is not None:
        q_dict["robot_uuid"] = robot_uuid
    if username is not None:
        q_dict["username"] = username
    return await FtpInfo_Pydantic.from_queryset(FtpInfo.filter(**q_dict))


@router.get(
    "/ftp_info/all", 
    response_model=List[FtpInfo_Pydantic],
    responses={404: {"model": HTTPNotFoundError}},
    description="get all ftp settings info"
)
async def get_all_ftp_info():
    return await FtpInfo_Pydantic.from_queryset(FtpInfo.all())


# @router.patch(
#     "/ftp_info/", 
#     response_model=FtpInfo_Pydantic, 
#     responses={404: {"model": HTTPNotFoundError}}
# )
# async def update_ftp_info(data: FtpInfoIn_Pydantic):
#     robot_uuid = data.robot_uuid
#     username = data.username
#     await FtpInfo.filter(robot_uuid=robot_uuid, username=username).update(**data.dict(exclude_unset=True))
#     return await FtpInfo_Pydantic.from_queryset_single(FtpInfo.get(robot_uuid=robot_uuid, username=username))


@router.post("/ftp_info/" , response_model=FtpInfo_Pydantic)
async def create_ftp_info(ftp_info: FtpInfoIn_Pydantic):
    ftp_info_dict = ftp_info.dict(exclude_unset=True)
    ftp_info_dict["mount_point"] = ftp_mount_point_manager.get_mount_point_path(ftp_info.robot_uuid, ftp_info.username)
    ftp_info_obj = await FtpInfo.create(**ftp_info_dict)
    return await FtpInfo_Pydantic.from_tortoise_orm(ftp_info_obj)

    
@router.delete("/ftp_info/", response_model=Status, responses={404: {"model": HTTPNotFoundError}})
async def delete_ftp_info(ftp_info: FtpInfoIn_Pydantic_Del):
    deleted_count = await FtpInfo.get(**ftp_info.dict(exclude_unset=True)).delete()
    if not deleted_count:
        raise HTTPException(status_code=404, detail=f"ftp info not found: robot_uuid={ftp_info.robot_uuid}, ftp username={ftp_info.username}")
    return Status(success=True, message=f"Deleted ftp info: robot_uuid uuid={ftp_info.robot_uuid}, ftp username={ftp_info.username}")