from typing import List, Dict, Optional, Union
from tortoise.contrib.pydantic import pydantic_model_creator
from features.virtual_robot import virtual_bot_manager, VirtualBotRecord
from models.status import Status
from features.token import verify_x_token
from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel


router = APIRouter(tags=["virtual_robot"])


class VisualBotConfig(BaseModel):
    name: str
    sudo_password: str
    password: Optional[str]
    expose_ports: Optional[List[int]]
    default_workspace: Optional[str]
    docker_image: Optional[str]
    command: Optional[str]
    
    
class VisualBotIn_Pydantic_Del(BaseModel):
    id: str


class ResponseMsg(Status):
    robot_info: Optional[VirtualBotRecord]


@router.post(
    "/v_bot/", 
    response_model=ResponseMsg,
    description="create virtual robot"
)
async def create_virtual_robot(
    data:VisualBotConfig, token_payload: dict = Depends(verify_x_token)
):
    user_uuid = token_payload["user_uuid"]
    kwargs = data.dict(exclude_unset=True)
    new_vBot = virtual_bot_manager.create(owner=user_uuid, **kwargs)
    if new_vBot is None:
        raise HTTPException(status_code=417, detail="create visual robot failed")
    else:
        return ResponseMsg(
            success=True, message="create visual robot success",
            robot_info=virtual_bot_manager.get_visual_robot_info(new_vBot.id)
        )


@router.delete(
    "/v_bot/", 
    response_model=Status,
    description="remove virtual robot"
)
async def del_virtual_robot(
    data:VisualBotIn_Pydantic_Del, 
    token_payload: dict = Depends(verify_x_token)
):
    success = virtual_bot_manager.destory(data.id)
    return Status(
        success=success, 
        message=f"remove visual robot {id} {'success' if success else 'failed'}"
    )


@router.get(
    "/v_bot/", 
    response_model=ResponseMsg,
    description="get virtual robot info by id"
)
async def get_vBot_info(
    id:str, token_payload: dict = Depends(verify_x_token)
):
    record = virtual_bot_manager.get_visual_robot_info(id)
    if record is None:
        raise HTTPException(
            status_code=417, 
            detail=f"isual robot [{id}] not exsist"
        )
    else:
        return ResponseMsg(
            success=True, 
            message="get visual robot info success",
            robot_info=record
        )


@router.get(
    "/v_bot/all", 
    response_model=List[VirtualBotRecord],
    description="get all virtual robots info"
)
async def all_vBot_info(token_payload: dict = Depends(verify_x_token)):
    return virtual_bot_manager.all_visual_robot_info()


@router.get(
    "/v_bot/my_vbot", 
    response_model=List[VirtualBotRecord],
    description="get virtual robots info by owner"
)
async def my_vBot_info(token_payload: dict = Depends(verify_x_token)):
    user_uuid = token_payload["user_uuid"]
    return virtual_bot_manager.filter_info_by_owner(user_uuid)