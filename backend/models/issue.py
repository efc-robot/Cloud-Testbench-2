from tortoise import fields, models
from tortoise.contrib.pydantic import pydantic_model_creator
import datetime


class Issue(models.Model):
    
    id = fields.IntField(pk=True, unique=True)
    time = fields.DatetimeField(default=datetime.datetime.now())
    uuid = fields.UUIDField(max_length=255, unique=True)
    reporter = fields.UUIDField(max_length=255)
    issue = fields.CharField(max_length=1023, null=True)
    solved = fields.BooleanField(default=False)
    
    class Meta:
        table = "issues"
        
    class PydanticMeta:
        exclude = ["id"]
        
    def __str__(self):
        return self.name
    

Issue_Pydantic = pydantic_model_creator(Issue, name="Issue")
IssueIn_Pydantic = pydantic_model_creator(Issue, name="IssueIn", exclude_readonly=True, exclude=["uuid", "time", "reporter"])