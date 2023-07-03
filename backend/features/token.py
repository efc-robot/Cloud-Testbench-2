import jwt
from datetime import datetime, timedelta
from fastapi import HTTPException, status, Header, Depends
from features.user import user_manager

SECRITE_KEY = 'my_secret_key'
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 600

# generate JWT token
def generate_token(data, expires_minutes=ACCESS_TOKEN_EXPIRE_MINUTES) -> str:
    exp_time = datetime.utcnow() + timedelta(minutes=expires_minutes)
    data.update({'exp': exp_time, 'iss': 'my_issuer'})
    token = jwt.encode(data, SECRITE_KEY, algorithm=ALGORITHM)
    return token.decode('utf-8')

# verify JWT token
def is_token_valid(token) -> bool:
    try:
        token_payload = jwt.decode(token, SECRITE_KEY, algorithms=[ALGORITHM], issuer='my_issuer')
    except:
        return False
    user_uuid = token_payload["user_uuid"]
    if user_manager.get(user_uuid) is None:
        return False
    else:
        return True

# decode JWT token
def decode_token(token) -> bool:
    try:
        decoded = jwt.decode(token, SECRITE_KEY, algorithms=[ALGORITHM], issuer='my_issuer')
        return decoded
    except:
        return None
    
async def get_token_header(Authorization: str = Header(...)):
    if not Authorization:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Token missing")
    return Authorization

async def verify_x_token(token: str = Depends(get_token_header)):
    if is_token_valid(token):
        token_data = decode_token(token)
        return token_data
    else:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Token invalid or expired")
    
# if __name__ == "__main__":
#     data = {
#         "username": "setsuna"
#     }
    
#     token = generate_token(data)
#     valid = is_token_valid(token)
#     data = decode_token(token)
#     print(data)