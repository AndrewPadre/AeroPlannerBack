# api_hud_stream.py
import time
import math
import asyncio
from typing import Callable, Iterable, Iterator, AsyncIterator, Union

import cv2
import numpy as np
from fastapi import APIRouter, Query, Request, Depends
from starlette.responses import StreamingResponse

from app.deps import get_mav, check_error
from core.services.mavlink_connection import MavlinkConnection
from core.rendering.hud import Hud, HUD_LAYOUT, TelemetryDTO


# ---------- Minimal MJPEG wrapper ----------
BytesIter = Union[Iterable[bytes], Iterator[bytes], AsyncIterator[bytes]]

class MjpegResponse(StreamingResponse):
    """Wrap an iterator of JPEG bytes into multipart/x-mixed-replace."""
    def __init__(self, frames: Callable[[], BytesIter], boundary: str = "frame"):
        async def _agen():
            head = f"--{boundary}\r\n".encode("ascii")
            ctype = b"Content-Type: image/jpeg\r\n"
            src = frames()
            # Support sync or async generators without exposing multipart in the route
            if hasattr(src, "__anext__"):  # async iterator
                async for jpg in src:
                    yield head; yield ctype
                    yield f"Content-Length: {len(jpg)}\r\n\r\n".encode("ascii")
                    yield jpg; yield b"\r\n"
            else:  # sync iterator
                for jpg in src:
                    yield head; yield ctype
                    yield f"Content-Length: {len(jpg)}\r\n\r\n".encode("ascii")
                    yield jpg; yield b"\r\n"
        super().__init__(_agen(), media_type=f"multipart/x-mixed-replace; boundary={boundary}")


# ---------- Single shared HUD (optionally call hud.stream_video(0) elsewhere) ----------
hud = Hud((800, 600), HUD_LAYOUT)
router = APIRouter()


# ---------- Single-client guard (cancels previous stream when a new one starts) ----------
class SingleClientStream:
    """Keeps a single active stream by cancelling the previous one via an asyncio.Event."""
    def __init__(self):
        self._lock = asyncio.Lock()
        self._stop_event: asyncio.Event | None = None

    async def start_new(self) -> asyncio.Event:
        # Cancel previous, create a new stop token
        async with self._lock:
            if self._stop_event is not None:
                self._stop_event.set()
            self._stop_event = asyncio.Event()
            return self._stop_event

single_client = SingleClientStream()


# ---------- Small helpers ----------
def encode_jpeg(bgr: np.ndarray, quality: int = 85) -> bytes:
    """Encode BGR to JPEG; guard against None/empty frames."""
    if bgr is None or not hasattr(bgr, "size") or bgr.size == 0:
        bgr = np.zeros((2, 2, 3), dtype=np.uint8)
    ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    return buf.tobytes() if ok else b"\xff\xd8\xff\xd9"


def simulate_telem(t: float) -> TelemetryDTO:
    """Smooth time-based telemetry for demo (replace with your backend later)."""
    roll  = 25.0 * math.sin(0.60 * t)               # ±25°
    pitch = 15.0 * math.sin(0.23 * t + 1.1)         # ±15°
    yaw   = (12.0 * t) % 360.0                      # deg/s
    spd   = 22.0 + 3.5 * math.sin(0.90 * t)
    alt   = 120.0 + 12.0 * math.sin(0.18 * t + 0.4)
    return TelemetryDTO(roll=roll, pitch=pitch, yaw=yaw, spd=spd, alt=alt)


# ---------- Route: MJPEG HUD stream (single-client) ----------
@router.get("/hud.mjpg")
async def hud_stream(
    request: Request,
    w: int = Query(800, ge=160, le=3840),
    h: int = Query(600, ge=120, le=2160),
    mav: MavlinkConnection=Depends(get_mav),
):
    """
    Single-client MJPEG stream. New requests cancel the previous stream (useful when size changes).
    """
    stop_event = await single_client.start_new()  # cancel previous stream if any

    async def frames():
        # Resize HUD once for the requested output size
        hud.resize((w, h))
        t0 = time.monotonic()

        try:
            while not stop_event.is_set():
                # Stop immediately if client disconnected
                if await request.is_disconnected():
                    break

                # 1) Telemetry (replace with your provider)
                t = time.monotonic() - t0
                roll = mav.get_hud_telemetry().get("roll", 0)
                pitch = mav.get_hud_telemetry().get("pitch", 0)
                telem = TelemetryDTO(roll=roll, pitch=pitch)
                # 2) Render HUD into internal buffer
                hud.render(telem)
                img = hud.img

                # 3) Encode & yield
                yield encode_jpeg(img)

                # 4) Simple pacing
                # next_tick += interval
                # delay = next_tick - time.monotonic()
                # if delay > 0:
                #     await asyncio.sleep(delay)
                # else:
                #     # If rendering lags, catch up to avoid spiral-of-death
                #     next_tick = time.monotonic()
        finally:
            # Place for cleanup if you open external resources later
            print()
            pass

    return MjpegResponse(frames)
