from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    HOST: str = "127.0.0.1"
    PORT: int = 3939
    API_PREFIX: str = "/api"
    MAVLINK_URL: str = "udp:0.0.0.0:14550"

settings = Settings()
