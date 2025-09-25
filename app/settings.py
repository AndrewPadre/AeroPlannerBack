from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    HOST: str = "127.0.0.1"
    PORT: int = 3939
    API_PREFIX: str = "/api"
    MAVLINK_URL: str = "udp:0.0.0.0:14552"
    GIMBAL_VIEW_PRO_URL: str = "192.168.30.119"
    GIMBAL_VIEW_PRO_PORT: int = 2000
    VIEW_PRO_HEARTBEAT: str = "55AADC04100014"

settings = Settings()
