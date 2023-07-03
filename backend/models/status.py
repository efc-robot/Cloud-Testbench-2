from pydantic import BaseModel


class Status(BaseModel):
    success: bool
    message: str