import uuid
from typing import List
from features.token import verify_x_token
from fastapi import APIRouter, HTTPException, Depends
from models.issue import Issue, Issue_Pydantic, IssueIn_Pydantic
from pydantic import BaseModel
from models.status import Status


router = APIRouter(tags=["issues"])


@router.post(
    "/issues/", 
    response_model=Issue_Pydantic,
    description="create issue record"
)
async def new_issue(
    data:IssueIn_Pydantic, token_payload: dict = Depends(verify_x_token)
):
    data_dict = data.dict(exclude_unset=True)
    user_uuid = uuid.UUID(token_payload["user_uuid"])
    data_dict["reporter"] = user_uuid
    data_dict["uuid"] = uuid.uuid1()
    print(data_dict)
    record = await Issue.create(**data_dict)
    return await Issue_Pydantic.from_tortoise_orm(record)


@router.get(
    "/issues/all", 
    response_model=List[Issue_Pydantic],
    description="get all issue record"
)
async def get_all_issues():
    return await Issue_Pydantic.from_queryset(Issue.all())


class Form_delIssue(BaseModel):
    uuid: str
    
@router.delete(
    "/issues/", 
    response_model=Status,
    description="remove virtual robot"
)
async def del_issue_record(
    uuid: str,
    token_payload: dict = Depends(verify_x_token)
):
    operator_uuid = token_payload["user_uuid"]
    operator_role = token_payload["role"]
    record = await Issue.get(uuid=uuid)
    reporter_uuid = record.reporter.__str__()
    if (operator_uuid != reporter_uuid) and (operator_role != "admin"):
        raise HTTPException(status_code=404, detail=f"delete issue failed, not your issue")
    elif not await Issue.filter(uuid=uuid).delete():
        raise HTTPException(status_code=404, detail=f"issue not found: uuid={uuid}")
    else:
        return Status(success=True, message=f"delete issue success,  uuid={uuid}")


# @router.get(
#     "/v_bot/", 
#     response_model=ResponseMsg,
#     description="get virtual robot info by id"
# )
# async def get_vBot_info(
#     id:str, token_payload: dict = Depends(verify_x_token)
# ):
#     record = virtual_bot_manager.get_visual_robot_info(id)
#     if record is None:
#         raise HTTPException(
#             status_code=417, 
#             detail=f"isual robot [{id}] not exsist"
#         )
#     else:
#         return ResponseMsg(
#             success=True, 
#             message="get visual robot info success",
#             robot_info=record
#         )