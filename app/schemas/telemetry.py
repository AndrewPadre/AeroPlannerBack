from pydantic import BaseModel, ConfigDict
from typing import Optional

class Telemetry(BaseModel):
    model_config = ConfigDict(from_attributes=True)  # Enable creating Telemery object from other object with exact fields. Use Telemetry.model_validate()
    t: float  # Timestamp
    roll: Optional[float] = None
    pitch: Optional[float] = None
    yaw: Optional[float] = None
    alt: Optional[float] = None
    lat: Optional[float] = None
    lon: Optional[float] = None
    mode: Optional[str] = None
