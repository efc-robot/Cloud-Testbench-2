from tortoise import fields, models
from tortoise.contrib.pydantic import pydantic_model_creator


class UserInfo(models.Model):
    
    id = fields.IntField(pk=True, unique=True)
    uuid = fields.UUIDField(max_length=255, unique=True)
    username = fields.CharField(max_length=128, unique=True)
    password = fields.CharField(max_length=128)
    role = fields.CharField(max_length=32, default="user")
    
    class Meta:
        table = "user_info"
        
    class PydanticMeta:
        exclude = ["id"]
        
    def __str__(self):
        return self.name
    

UserInfo_Pydantic = pydantic_model_creator(UserInfo, name="UserInfo")
UserInfoIn_Pydantic = pydantic_model_creator(UserInfo, name="UserInfoIn", exclude_readonly=True, optional=["uuid"])