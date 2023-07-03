import uuid

from fastapi import APIRouter, HTTPException, Depends
from tortoise.contrib.fastapi import HTTPNotFoundError

from typing import List
from pydantic import BaseModel
from models.status import Status
from models.user import UserInfo_Pydantic, UserInfoIn_Pydantic, UserInfo

from features.token import verify_x_token
from features.user import user_manager


router = APIRouter(tags=["user"])


@router.get(
    "/user/", 
    response_model=UserInfo_Pydantic, 
    responses={404: {"model": HTTPNotFoundError}}
)
async def get_user(uuid: str):
    return await UserInfo_Pydantic.from_queryset_single(UserInfo.get(uuid=uuid))


@router.patch(
    "/user/", 
    response_model=UserInfo_Pydantic, 
    responses={404: {"model": HTTPNotFoundError}}
)
async def update_user(robot_info: UserInfoIn_Pydantic):
    await UserInfo.filter(uuid=robot_info.uuid).update(**robot_info.dict(exclude_unset=True, exclude={"uuid"}, exclude_none=True))
    return await UserInfo_Pydantic.from_queryset_single(UserInfo.get(uuid=robot_info.uuid))


@router.post("/user/" , response_model=UserInfo_Pydantic)
async def create_user(robot_info: UserInfoIn_Pydantic):
    robot_info_dict = robot_info.dict(exclude_unset=True, exclude_none=True)
    robot_info_dict["uuid"] = uuid.uuid1()
    robot_info_obj = await UserInfo.create(**robot_info_dict)
    return await UserInfo_Pydantic.from_tortoise_orm(robot_info_obj)

    
@router.delete("/user/", response_model=Status, responses={404: {"model": HTTPNotFoundError}})
async def delete_user(uuid: str):
    deleted_count = await UserInfo.filter(uuid=uuid).delete()
    if not deleted_count:
        raise HTTPException(status_code=404, detail=f"robot not found: uuid={uuid}")
    return Status(success=True, message=f"Deleted robot, uuid={uuid}")


class Form_acquire_robot(BaseModel):
    robot_uuid: str
    robot_username: str
    password: str
    workspace: str

@router.post("/user/acquire_robot" , response_model=Status)
async def user_acquire_robot(data: Form_acquire_robot, token_payload: dict = Depends(verify_x_token)):
    user_uuid = username=token_payload["user_uuid"]
    user = user_manager.get(user_uuid)
    success = user.acquire_robot(data.robot_uuid, data.robot_username, data.password, data.workspace)
    if success:
        username = token_payload["username"]
        return Status(success=success, message=f"robot '{data.robot_uuid}_{data.robot_username}' allocated to user '{username}'")
    else:
        return Status(success=success, message=f"user '{username}' acquire robot '{data.robot_uuid}_{data.robot_username}' failed")

@router.get(
    "/user/all", 
    response_model=List[UserInfo_Pydantic],
    responses={404: {"model": HTTPNotFoundError}},
    description="get all user info"
)
async def get_all_user_info():
    return await UserInfo_Pydantic.from_queryset(UserInfo.all())