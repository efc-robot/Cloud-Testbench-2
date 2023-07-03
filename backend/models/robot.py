from tortoise import fields, models
from tortoise.contrib.pydantic import pydantic_model_creator


class RobotInfo(models.Model):
    
    id = fields.IntField(pk=True, unique=True)
    uuid = fields.UUIDField(max_length=255, unique=True)
    name = fields.CharField(max_length=255, null=True, default="new robot")
    type = fields.CharField(max_length=64, null=True)
    ip = fields.CharField(max_length=64, null=True)
    
    class Meta:
        table = "robot_info"
        
    class PydanticMeta:
        exclude = ["id"]
        
    def __str__(self):
        return self.name
    

RobotInfo_Pydantic = pydantic_model_creator(RobotInfo, name="RobotInfo")
RobotInfoIn_Pydantic = pydantic_model_creator(RobotInfo, name="RobotInfoIn", exclude_readonly=True)
RobotInfoIn_Pydantic_Post = pydantic_model_creator(RobotInfo, name="RobotInfoIn_Post", exclude_readonly=True, exclude=["uuid"])