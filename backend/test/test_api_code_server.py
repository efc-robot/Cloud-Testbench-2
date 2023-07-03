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
async def test_create_code_server_workspace(client: AsyncClient):
    print("\n[test_create_code_server_workspace]")
    json_data = {
        "uuid_user": "8a940796-ac48-11ed-bbda-8565430a6f0c",
        "uuid_robot": "mock_uuid"
    }
    response = await client.post("/code_server/", json=json_data)
    assert response.status_code == 200, response.text
    # global new_robot_uuid
    # new_robot_uuid = response.json()["uuid"]
    # print(f"new robor created, uuid:[{new_robot_uuid}]")