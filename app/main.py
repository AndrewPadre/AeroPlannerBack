from fastapi import FastAPI
from app.settings import settings
from app.api import connection, telemetry

app = FastAPI(title="Aero Planner Backend")
app.include_router(connection.router, prefix=f"{settings.API_PREFIX}", tags=["connection"])
app.include_router(telemetry.router,  prefix=f"{settings.API_PREFIX}", tags=["telemetry"])
