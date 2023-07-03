import logging
logging.basicConfig(
    level=logging.DEBUG,
    format='[%(asctime)s] [%(levelname)s] [%(filename)s:%(lineno)s]: %(message)s',
    handlers=[
        logging.FileHandler('remote_bot_system.log'),
        logging.StreamHandler()
    ]
)

import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from tortoise.contrib.fastapi import register_tortoise

from api import user, robot, ftp_info, login, files, virtual_robot, issues


app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


register_tortoise(
    app,
    db_url="mysql://root:root@127.0.0.1:3306/cloud_test_bench",
    modules={"models": ["models.robot", "models.user", "models.ftp", "models.issue"]},
    generate_schemas=True,
    add_exception_handlers=True,
)


app.include_router(robot.router)
app.include_router(user.router)
# app.include_router(code_server.router)
app.include_router(ftp_info.router)
app.include_router(login.router)
app.include_router(files.router)
app.include_router(virtual_robot.router)
app.include_router(issues.router)


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)