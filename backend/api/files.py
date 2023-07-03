from fastapi import APIRouter
from fastapi.responses import FileResponse


router = APIRouter(tags=["files"])

@router.get("/files/{file_path:path}")
async def read_file(file_path: str):
    return FileResponse(f"public/{file_path}")