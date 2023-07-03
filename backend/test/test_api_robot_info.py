# mypy: no-disallow-untyped-decorators
# pylint: disable=E0611,E0401
import pytest
from asgi_lifespan import LifespanManager
from httpx import AsyncClient
from main import app


@pytest.fixture(scope="module")
def anyio_backend():
    return "asyncio"


@pytest.fixture(scope="module")
async def client():
    async with LifespanManager(app):
        async with AsyncClient(app=app, base_url="http://test") as c:
            yield c
            
            
new_robot_uuid = ''


@pytest.mark.anyio
async def test_create_robot_info(client: AsyncClient):
    print("\n[test_create_robot_info]")
    robot_info = {
        "name": "mock bot 001",
        "type": "mock bot",
        "ip": "192.168.124.137",
        "ftp_service_port": 21,
        "ftp_username": "haro",
        "ftp_password": "1",
        "ftp_workspace": "/Documents/bot_workspace"
    }
    response = await client.post("/robot_info/", json=robot_info)
    assert response.status_code == 200, response.text
    global new_robot_uuid
    new_robot_uuid = response.json()["uuid"]
    print(f"new robor created, uuid:[{new_robot_uuid}]")


@pytest.mark.anyio
async def test_get_robot_info_by_id(client: AsyncClient):
    print("\n[test_get_robot_info_by_id]")
    global new_robot_uuid
    response = await client.get(f"/robot_info/?uuid={new_robot_uuid}")
    assert response.status_code == 200, response.text
    data = response.json()
    print(f"get robot info success, robot info: {data}")


@pytest.mark.anyio
async def test_update_robot_info(client: AsyncClient):
    print("\n[test_update_robot_info]")
    global new_robot_uuid
    robot_info = {
        "name": "new mock robot in test",
        "type": "mock bot",
        "ip": "192.168.124.137",
        "ftp_service_port": 21,
        "ftp_username": "haro",
        "ftp_password": "1",
        "ftp_workspace": "/Documents/bot_workspace"
    }
    response = await client.put(f"/robot_info/?uuid={new_robot_uuid}", json=robot_info)
    assert response.status_code == 200, response.text
    data = response.json()
    assert data["name"] == robot_info["name"]
    print(f"robot info updated, robot info: {data}")


@pytest.mark.anyio
async def test_delete_robot(client: AsyncClient):
    print("\n[test_delete_robot]")
    global new_robot_uuid
    response = await client.delete(f"/robot_info/?uuid={new_robot_uuid}")
    assert response.status_code == 200, response.text
    print(f"robot info deleted, uuid:[{new_robot_uuid}]")