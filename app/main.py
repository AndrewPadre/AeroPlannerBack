from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.settings import settings
from app.api import connection, telemetry, joystick, gimbal_view_pro, commands

origins = [
    "http://localhost:1420",  
    "http://127.0.0.1:1420",  
]

app = FastAPI(title="Aero Planner Backend")
app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,      
    allow_credentials=True,
    allow_methods=["*"],       
    allow_headers=["*"],      
)

app.include_router(connection.router, prefix=f"{settings.API_PREFIX}", tags=["connection"])
app.include_router(telemetry.router,  prefix=f"{settings.API_PREFIX}", tags=["telemetry"])
app.include_router(commands.router, prefix=f"{settings.API_PREFIX}", tags=["commands"])
app.include_router(joystick.router,  prefix=f"{settings.API_PREFIX}/joystick", tags=["joystick"])
app.include_router(gimbal_view_pro.router,  prefix=f"{settings.API_PREFIX}", tags=["GimbalViewPro"])
