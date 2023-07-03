from tortoise import fields, models
from tortoise.contrib.pydantic import pydantic_model_creator


class FtpInfo(models.Model):
    
    id = fields.IntField(pk=True, unique=True)
    robot_uuid = fields.UUIDField()
    username = fields.CharField(max_length=128)
    password = fields.CharField(max_length=128)
    remote_dir = fields.CharField(max_length=512, default="/")
    mount_point = fields.CharField(max_length=512, unique=True)
    port = fields.IntField(null=True, default=21)
    
    class Meta:
        table = "ftp_info"
        unique_together = ("robot_uuid", "username")
        
    class PydanticMeta:
        exclude = ["id"]
        
    def __str__(self):
        return self.name
    

FtpInfo_Pydantic = pydantic_model_creator(FtpInfo, name="FtpInfo")
FtpInfoIn_Pydantic = pydantic_model_creator(FtpInfo, name="FtpInfoIn_post", exclude_readonly=True, exclude=["mount_point"])
FtpInfoIn_Pydantic_Get = pydantic_model_creator(
    FtpInfo, 
    name="FtpInfoIn_get", 
    exclude_readonly=True, 
    exclude=["mount_point", "password", "remote_dir"],
    optional=["robot_uuid", "username"]
)
FtpInfoIn_Pydantic_Del = pydantic_model_creator(
    FtpInfo, 
    name="FtpInfoIn_del", 
    exclude_readonly=True, 
    exclude=["password", "remote_dir", "mount_point", "port"],
)