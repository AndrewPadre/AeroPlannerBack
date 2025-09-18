
from fastapi import Depends, APIRouter
from app.deps import get_gimbal_view_pro
from core.services.gimbal_view_pro import GimbalViewPro

router = APIRouter()

@router.get("/gimbalViewPro")
def gimbal_view_pro(gimbal: GimbalViewPro = Depends(get_gimbal_view_pro)):
    return gimbal.gimbal_data_dict

