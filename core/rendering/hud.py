# hud_refactored.py
# Variant A: Component-based HUD with anchors/offsets layout and OpenCV renderer

from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Callable, Tuple, Optional, List, Dict
import cv2
import numpy as np

@dataclass(frozen=True)
class TelemetryDTO:
    # Attitude / heading
    roll: float = 0.0
    pitch: float = 0.0

    yaw: float = 0.0  
    gps_yaw: float = 0.0
    yaw_to_wp: float = 0.0
    gps_raw_yaw: float = 0.0 

    # Speeds / altitude (HUD expects spd & alt)
    spd: float = 0.0           # airspeed for HUD
    alt: float = 0.0           # preferred altitude for HUD (baro first)

    lat: Optional[float] = None
    lon: Optional[float] = None
    alt_baro: Optional[float] = None
    alt_gps: Optional[float] = None
    groundspeed: Optional[float] = None
    airspeed: Optional[float] = None
    
    raw_lat: Optional[float] = None
    raw_lon: Optional[float] = None

    # Power/etc. (extend as needed)
    bat_voltage: Optional[float] = None


# =========================
# Styling / Theming
# =========================
@dataclass
class Style:
    """Visual style shared across widgets."""
    line: Tuple[int, int, int] = (255, 255, 255)
    accent: Tuple[int, int, int] = (0, 220, 255)
    box_border: int = 2
    font_scale_label: float = 0.55
    font_scale_value: float = 0.75
    font_thickness_label: int = 1

    # Lines thickness
    t_minor: int = 2
    t_major: int = 2

    # Center reticle
    reticle_radius: int = 14
    reticle_thickness: int = 1
    reticle_cross_half: int = 6
    
    # Horizon fill colors (BGR)
    sky_color: Tuple[int, int, int] = (127, 54, 7)   # light blue-ish sky
    ground_color: Tuple[int, int, int] = (30, 70, 100) # earthy brown
    


# =========================
# Layout primitives
# =========================
AnchorH = Tuple[str, str]  # ("top"|"center"|"bottom", "left"|"center"|"right")

@dataclass
class LayoutBox:
    """Declarative layout: anchor + size + offset."""
    anchor: AnchorH = ("center", "center")
    size: Optional[Tuple[int, int]] = None  # (w,h) where applicable
    offset: Tuple[str, str] = ("0", "0")        # (dx, dy) from anchor


def resolve_anchor_point(frame_w: int, frame_h: int, anchor: AnchorH) -> Tuple[int, int]:
    """Compute anchor pixel position for a given frame size."""
    vy, vx = anchor
    if vx == "left":
        ax = 0
    elif vx == "center":
        ax = frame_w // 2
    elif vx == "right":
        ax = frame_w
    else:
        raise ValueError(f"Bad horizontal anchor: {vx}")

    if vy == "top":
        ay = 0
    elif vy == "center":
        ay = frame_h // 2
    elif vy == "bottom":
        ay = frame_h
    else:
        raise ValueError(f"Bad vertical anchor: {vy}")

    return ax, ay


# =========================
# Tiny renderer abstraction
# =========================
class CvRenderer:
    """Thin wrapper around OpenCV primitives to keep widget code clean."""
    def __init__(self, img: np.ndarray):
        self.img = img

    def line(self, p1, p2, color, thickness):
        cv2.line(self.img, p1, p2, color, thickness, cv2.LINE_AA)

    def rect(self, p1, p2, color, thickness, filled: bool = False):
        cv2.rectangle(self.img, p1, p2, color, -1 if filled else thickness, cv2.LINE_AA)

    def circle(self, center, radius, color, thickness):
        cv2.circle(self.img, center, radius, color, thickness, cv2.LINE_AA)

    def fill_poly(self, pts: np.ndarray, color):
        cv2.fillConvexPoly(self.img, pts, color)

    def text(self, text, org, scale, color, thickness):
        # Draw outlined text for contrast
        # cv2.putText(self.img, text, (org[0] + 1, org[1] + 1), cv2.FONT_HERSHEY_SIMPLEX,
                    # scale, (0, 0, 0), thickness + 5, cv2.LINE_AA)
        cv2.putText(self.img, text, org, cv2.FONT_HERSHEY_SIMPLEX,
                    scale, color, thickness, cv2.LINE_AA)


# =========================
# Base widget
# =========================
class BaseWidget:
    """Base class: handles anchor + offset → absolute placement."""
    def __init__(self, layout: LayoutBox, style: Style):
        self.layout = layout
        self.style = style
        self._frame_size = (0, 0)  # (w,h)
        self._anchor_px = (0, 0)

    def layout_frame(self, frame_w: int, frame_h: int):
        """Compute anchor pixel for the current frame; cache internally."""
        self._frame_size = (frame_w, frame_h)
        ax, ay = resolve_anchor_point(frame_w, frame_h, self.layout.anchor)
        dx, dy = self.layout.offset
        # if offset is string - transform to int
        if isinstance(dx, str):
            if dx.endswith("px"):
                dx = int(dx[:-2])
            elif dx.endswith("%"):
                percent = int(dx[:-1])
                dx = self._frame_size[0] * percent // 100
            else:
                raise Exception("The value is not supported")
        
        if isinstance(dy, str):
            if dy.endswith("px"):
                dy = int(dy[:-2])
            elif dy.endswith("%"):
                percent = int(dy[:-1])
                dy = self._frame_size[1] * percent // 100
            else:
                raise Exception("The value is not supported")
        self._anchor_px = (ax + dx, ay + dy)

    def draw(self, r: CvRenderer, telem: TelemetryDTO):
        """Implement in derived classes: uses self._anchor_px and self._frame_size."""
        raise NotImplementedError


# =========================
# Concrete widgets
# =========================
class CenterReticle(BaseWidget):
    """Small ring + very short cross centered on anchor."""
    def draw(self, r: CvRenderer, telem: TelemetryDTO, thickness=2):
        frame_w, frame_h = self._frame_size
        if frame_h < 400:
            thickness = 1
        else:
            thickness = 2
        cx, cy = self._anchor_px
        s = self.style
        r.circle((cx, cy), s.reticle_radius, s.accent, thickness)
        r.line((cx - s.reticle_cross_half, cy), (cx + s.reticle_cross_half, cy), s.accent, thickness)
        r.line((cx, cy - s.reticle_cross_half), (cx, cy + s.reticle_cross_half), s.accent, thickness)


class CenterWings(BaseWidget):
    """Horizontal 'goalpost' wings with small brackets and fangs."""
    def __init__(self, layout: LayoutBox, style: Style,
                 left_margin=80, right_margin=80,
                 center_spacing=50, arm=200, inner=70,
                 vertical_gap=10, vertical_line=20, lift=0,
                 thickness_outer=2, thickness_inner=2, thickness_fang=2):
        super().__init__(layout, style)
        # Store geometry as configuration
        self.left_margin = left_margin
        self.right_margin = right_margin
        self.center_spacing = center_spacing
        self.arm = arm
        self.inner = inner
        self.vertical_gap = vertical_gap
        self.vertical_line = vertical_line
        self.lift = lift
        self.thickness_outer = thickness_outer
        self.thickness_inner = thickness_inner
        self.thickness_fang = thickness_fang

    def draw(self, r: CvRenderer, telem: TelemetryDTO):
        # Reuse original logic but computed from anchor + frame size
        frame_w, frame_h = self._frame_size

        if frame_h < 400:
            self.thickness_outer = 1
            self.thickness_inner = 1
            self.thickness_fang = 1
        else:
            self.thickness_outer = 2
            self.thickness_inner = 2
            self.thickness_fang = 2

        cx, cy = self._anchor_px
        cy = cy - self.lift
        color = self.style.accent

        left_end = self.left_margin
        right_end = frame_w - self.right_margin

        # Outer long wings
        r.line((left_end, cy), (cx - self.arm, cy), color, self.thickness_outer)
        r.line((cx + self.arm, cy), (right_end, cy), color, self.thickness_outer)

        # Inner short segments (lower)
        r.line((cx - self.inner - self.center_spacing, cy + self.vertical_gap),
               (cx - self.center_spacing, cy + self.vertical_gap), color, self.thickness_inner)
        r.line((cx + self.center_spacing, cy + self.vertical_gap),
               (cx + self.inner + self.center_spacing, cy + self.vertical_gap), color, self.thickness_inner)

        # Lower fangs
        r.line((cx - self.center_spacing, cy + self.vertical_gap),
               (cx - self.center_spacing, cy + self.vertical_gap + self.vertical_line), color, self.thickness_fang)
        r.line((cx + self.center_spacing, cy + self.vertical_gap),
               (cx + self.center_spacing, cy + self.vertical_gap + self.vertical_line), color, self.thickness_fang)

        # Upper short segments
        r.line((cx - self.inner - self.center_spacing, cy - self.vertical_gap),
               (cx - self.center_spacing, cy - self.vertical_gap), color, self.thickness_inner)
        r.line((cx + self.center_spacing, cy - self.vertical_gap),
               (cx + self.inner + self.center_spacing, cy - self.vertical_gap), color, self.thickness_inner)

        # Upper fangs
        r.line((cx - self.center_spacing, cy - self.vertical_gap),
               (cx - self.center_spacing, cy - (self.vertical_gap + self.vertical_line)), color, self.thickness_fang)
        r.line((cx + self.center_spacing, cy - self.vertical_gap),
               (cx + self.center_spacing, cy - (self.vertical_gap + self.vertical_line)), color, self.thickness_fang)


class PitchLadder(BaseWidget):
    """Vertical pitch ladder crossing the anchor (0° at anchor.y)."""
    def __init__(self, layout: LayoutBox, style: Style,
                 pixels_per_deg=6, min_deg=-30, max_deg=30,
                 tick_step=5, major_every=10,
                 half_len_minor=15, half_len_major=30,
                 zero_gap=20, y_margin=50,
                 label_scale=0.55, label_dx=8, label_dy=5,
                 positive_pitch_moves_down=True, clamp_to_range=False):
        super().__init__(layout, style)
        self.pixels_per_deg = pixels_per_deg
        self.min_deg = min_deg
        self.max_deg = max_deg
        self.tick_step = tick_step
        self.major_every = major_every
        self.half_len_minor = half_len_minor
        self.half_len_major = half_len_major
        self.zero_gap = zero_gap
        self.y_margin = y_margin
        self.label_scale = label_scale
        self.label_dx = label_dx
        self.label_dy = label_dy
        self.positive_pitch_moves_down = positive_pitch_moves_down
        self.clamp_to_range = clamp_to_range

    def draw(self, r: CvRenderer, telem: TelemetryDTO):
        frame_w, frame_h = self._frame_size
        cx, cy = self._anchor_px
        s = self.style

        # --- Simple dynamic scaling by current frame size ---
        # Reference canvas you designed for (tweak if needed)
        REF_W, REF_H = 600, 600

        # Scale factors clamped to reasonable bounds
        scale_len   = max(0.4, min(1.6, frame_w / REF_W)) * 0.9 # affects line length (width-driven)
        scale_thick = max(0.5, min(1, frame_h / REF_H))  # affects thickness (height-driven)
        scale_ppd   = max(0.5, min(0.7, frame_h / REF_H))  # affects pixels_per_deg (height-driven)
        label_scale = max(0.3, min(0.55, frame_h / REF_H) )


        # Scaled half-lengths; keep inside the canvas
        half_len_minor = max(2, min(frame_w // 2 - 6, int(round(self.half_len_minor * scale_len))))
        half_len_major = max(2, min(frame_w // 2 - 6, int(round(self.half_len_major * scale_len))))

        # Scaled thicknesses (never 0)
        thick_minor = max(1, int(round(s.t_minor * scale_thick)))
        thick_major = max(1, int(round(s.t_major * scale_thick)))

        # Scaled vertical density of the ladder
        pixels_per_deg = max(2, int(round(self.pixels_per_deg * scale_ppd)))

        pitch_deg = telem.pitch
        if self.clamp_to_range:
            pitch_deg = max(self.min_deg, min(self.max_deg, pitch_deg))

        sign = 1 if self.positive_pitch_moves_down else -1
        shift = int(round(pitch_deg * pixels_per_deg)) * sign

        for deg in range(self.min_deg, self.max_deg + 1, self.tick_step):
            y = cy - int(round(deg * pixels_per_deg)) + shift

            # Keep within vertical safe margins
            if y < self.y_margin or y > frame_h - self.y_margin:
                continue

            is_major = (deg % self.major_every == 0)
            half  = half_len_major if is_major else half_len_minor
            thick = thick_major    if is_major else thick_minor

            if deg == 0:
                # zero line with center gap
                r.line((cx - half, y), (cx - self.zero_gap, y), s.line, thick)
                r.line((cx + self.zero_gap, y), (cx + half,  y), s.line, thick)
            else:
                r.line((cx - half, y), (cx + half, y), s.line, thick)

            # if is_major and deg != 0:
                # r.text(f"{deg:+}", (cx + half + self.label_dx, y + self.label_dy),
                #        scale=label_scale, color=s.line, thickness=s.font_thickness_label)



class RollArc(BaseWidget):
    """Top roll arc with ticks and a pointer. Anchor is typically ("top","center")."""
    def __init__(self, layout: LayoutBox, style: Style,
                 span=80, tick_step=10, major_step=30,
                 tick_len_minor=10, tick_len_major=18,
                 thickness=2, base_deg=270, radius_max=220, radius_min=10):
        super().__init__(layout, style)
        self.span = span
        self.tick_step = tick_step
        self.major_step = major_step
        self.tick_len_minor = tick_len_minor
        self.tick_len_major = tick_len_major
        self.thickness = thickness
        self.base_deg = base_deg
        self.radius_max = radius_max
        self.radius_min = radius_min
        self.old_roll = 0
        self.old_roll_count = 0
        self.new_roll_count = 0

    def draw(self, r: CvRenderer, telem: TelemetryDTO):
        frame_w, frame_h = self._frame_size
        cx, cy = self._anchor_px
        s = self.style
        roll_deg = telem.roll
        if roll_deg == self.old_roll:
            self.old_roll_count += 1
        else:
            self.new_roll_count += 1
        # print(self.old_roll_count, " --- ", self.new_roll_count)

        self.old_roll = roll_deg


        # Geometry: choose radius that fits the frame
        radius = min(int(frame_w / 3.7), self.radius_max)
        radius = min(radius, int(frame_h / 3.7))
        radius = max(radius, self.radius_min)
        if radius < 120:
            self.thickness = 1
        elif radius > 120:
            self.thickness = 2
        cy = cy + radius

        # Arc boundaries (OpenCV ellipse uses degrees from +X, CCW)
        start_angle = self.base_deg - self.span
        end_angle = self.base_deg + self.span
        
        cv2.ellipse(r.img, (cx, cy), (radius, radius),
                    0, start_angle, end_angle, s.line, self.thickness, cv2.LINE_AA)

        # Ticks
        for d in range(-self.span, self.span + 1, self.tick_step):
            ang = math.radians(self.base_deg + d)
            is_major = (d % self.major_step == 0)
            r_outer = radius + 2
            r_inner = radius - (self.tick_len_major if is_major else self.tick_len_minor) + 2

            x2 = int(cx + r_outer * math.cos(ang))
            y2 = int(cy + r_outer * math.sin(ang))
            x1 = int(cx + r_inner * math.cos(ang))
            y1 = int(cy + r_inner * math.sin(ang))

            r.line((x1, y1), (x2, y2), s.line, self.thickness)

        # Pointer (triangle), clamped to visible span
        rd = max(-self.span, min(self.span, roll_deg))
        rr = math.radians(self.base_deg + rd)
        tip   = (int(cx + (radius - 2)  * math.cos(rr)), int(cy + (radius - 2)  * math.sin(rr)))
        left  = (int(cx + (radius - 14) * math.cos(rr + 0.1)), int(cy + (radius - 14) * math.sin(rr + 0.1)))
        right = (int(cx + (radius - 14) * math.cos(rr - 0.1)), int(cy + (radius - 14) * math.sin(rr - 0.1)))
        r.fill_poly(np.array([tip, left, right], np.int32), s.accent)

        # Static label


class CompactBox(BaseWidget):
    """Generic small box with label + dynamic value function."""
    def __init__(self, layout: LayoutBox, style: Style,
                 label: str, value_fn: Callable[[TelemetryDTO], str],
                 box_size: Tuple[int, int] = (110, 44),
                 label_scale_override=None):
        super().__init__(layout, style)
        self.label = label
        self.value_fn = value_fn
        self.box_w, self.box_h = box_size
        self.label_scale_value = label_scale_override
        self.thickness_value = 2
        if self.label_scale_value:
            # self.thickness_value = 1  # if label scale override, for the mode
            pass
        else:
            self.label_scale_value = style.font_scale_label

    def draw(self, r: CvRenderer, telem: TelemetryDTO):
        s = self.style
             
        x0, y0 = self._anchor_px
        
        MIN_W, MIN_H = 350, 350
        MAXW, MAX_H = 700, 700
        
        scale_w = max(0.5, min(1, self._frame_size[0] / MIN_W))
        scale_h = max(0.7, min(1, self._frame_size[1] / MIN_H))
        
        scale = min(scale_w, scale_h)
        box_w = int(self.box_w * scale)
        box_h = int(self.box_h * scale)
        

        # Treat anchor point as top-left corner for the box
        x1, y1 = x0 + box_w, y0 + box_h
        if self.layout.anchor[1] == "right":
            x0 = x0 - box_w
            x1 = x1 - box_w

        if self.layout.anchor[1] == "center":
            x0 = x0 - box_w // 2
            x1 = x1 - box_w // 2

        if self.layout.anchor[0] == "bottom":
            y0 = y0 - box_h
            y1 = y1 - box_h

        if self.layout.anchor[0] == "center":
            y0 = y0 - box_h // 2
            y1 = y1 - box_h // 2

        # Background + border
        r.rect((x0, y0), (x1, y1), (0, 0, 0), s.box_border, filled=True)
        r.rect((x0, y0), (x1, y1), s.line, s.box_border, filled=False)


        # Texto
        r.text(self.label, (x0 + int(10 * scale), y0 + int(18 * scale)), s.font_scale_label * scale, s.line, int(s.font_thickness_label * scale))
        r.text(self.value_fn(telem), (x0 + int(10 * scale), y0 + int(38 * scale)), self.label_scale_value * scale, s.accent, int(self.thickness_value * scale))
        

class VideoBackground(BaseWidget):
    """
    Background video feed widget.
    - Reads frames from a cv2.VideoCapture.
    - Renders as a full-frame 'cover' (preserves aspect ratio; crops edges if needed).
    - Optional horizontal mirroring (useful for front-facing cameras).
    """
    def __init__(self, layout: LayoutBox, style: Style,
                 cap: cv2.VideoCapture,
                 mirror: bool = False,
                 stretch_video=False):
        super().__init__(layout, style)
        self.cap = cap
        self.mirror = mirror
        self._stretch_video=stretch_video
        
    def set_stretch_video(self, stretch_video):
        self._stretch_video = stretch_video

    @staticmethod
    def _resize_cover(src: np.ndarray, dst_w: int, dst_h: int) -> np.ndarray:
        """Scale src to cover dst size, preserving aspect ratio; then center-crop."""
        sh, sw = src.shape[:2]
        if sw == 0 or sh == 0 or dst_w == 0 or dst_h == 0:
            return None

        # Compute uniform scale so that both dimensions are >= target
        scale = max(dst_w / sw, dst_h / sh)
        new_w, new_h = int(sw * scale), int(sh * scale)
        resized = cv2.resize(src, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        # Center-crop to target size
        x0 = (new_w - dst_w) // 2
        y0 = (new_h - dst_h) // 2
        return resized[y0:y0 + dst_h, x0:x0 + dst_w]

    def draw(self, r: CvRenderer, telem: TelemetryDTO):
        # Try to grab the next frame; on failure, keep previous content.
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return

        # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        # Optional mirror
        if self.mirror:
            frame = cv2.flip(frame, 1)

        # --- CRITICAL CHANGE: derive target size from the renderer buffer, not from self._frame_size ---
        dst_h, dst_w = r.img.shape[:2]   # authoritative target for THIS frame

        # Fit/crop to the exact target size
        fitted = None
        if self._stretch_video:
            fitted = cv2.resize(frame, (dst_w, dst_h), interpolation=cv2.INTER_LINEAR)
        else:
            fitted = self._resize_cover(frame, dst_w, dst_h)
            if fitted is None:
                return

        # Safety: if shapes mismatch for any reason, crop to intersection (shouldn't happen now)
        fh, fw = fitted.shape[:2]
        if fh != dst_h or fw != dst_w:
            print("Sizes don't match")
            h = min(dst_h, fh)
            w = min(dst_w, fw)
            r.img[:h, :w] = fitted[:h, :w]
            return

        # Fast path: exact match
        r.img[:] = fitted


# =========================
# New widget: Artificial Horizon
# =========================
class ArtificialHorizon(BaseWidget):
    """
    Artificial horizon with sky/ground fill.
    - Rotates with roll and shifts with pitch.
    - Fills the top half-plane (sky) and bottom half-plane (ground).
    """
    def __init__(self, layout: LayoutBox, style: Style,
                 pixels_per_deg=6,            # should match PitchLadder for perfect sync
                 thickness=2,                 # line thickness for the horizon
                 length_factor=2.2,           # line length vs frame diagonal
                 center_tick_len=16,          # tick length in px
                 ):
        super().__init__(layout, style)
        self.pixels_per_deg = pixels_per_deg
        self.thickness = thickness
        self.length_factor = length_factor
        self.center_tick_len = center_tick_len

    def draw(self, r: CvRenderer, telem: TelemetryDTO):
        # Anchor is the aircraft reference (reticle center).
        frame_w, frame_h = self._frame_size
        cx, cy = self._anchor_px
        s = self.style
        REF_H = 600
        scale_ppd = max(0.5, min(0.7, frame_h / REF_H))  # affects pixels_per_deg (height-driven)
        pixels_per_deg = max(2, int(round(self.pixels_per_deg * scale_ppd)))
        # --- Direction vectors ---
        # v: along the horizon line; n: normal to the line (points "down" at roll=0).
        ang = math.radians(telem.roll) * -1
        vx, vy = math.cos(ang), math.sin(ang)
        nx, ny = -math.sin(ang), math.cos(ang)

        # --- Pitch offset (match PitchLadder convention) ---
        offset = int(round(telem.pitch * pixels_per_deg))

        # Base point on the horizon (shifted by pitch along the normal)
        x0 = int(cx + nx * offset)
        y0 = int(cy + ny * offset)

        # Choose big extents so that polygons cover the whole frame (no gaps).
        diag = int(math.hypot(frame_w, frame_h))
        L = int(self.length_factor * diag)      # along the line
        M = int(3.0 * diag)                     # along the normal to cover half-planes

        # Two points on the horizon line (far apart)
        p1 = (int(x0 + vx * L), int(y0 + vy * L))
        p2 = (int(x0 - vx * L), int(y0 - vy * L))

        # Build two convex quads:
        # - SKY is the half-plane opposite to the normal (towards -n)
        # - GROUND is the half-plane along the normal (towards +n)
        sky = np.array([
            (int(p1[0] - nx * M), int(p1[1] - ny * M)),
            (int(p2[0] - nx * M), int(p2[1] - ny * M)),
            (p2[0], p2[1]),
            (p1[0], p1[1]),
        ], dtype=np.int32)

        ground = np.array([
            (int(p1[0] + nx * M), int(p1[1] + ny * M)),
            (int(p2[0] + nx * M), int(p2[1] + ny * M)),
            (p2[0], p2[1]),
            (p1[0], p1[1]),
        ], dtype=np.int32)

        # --- Draw fills first (background layer for the HUD) ---
        r.fill_poly(sky, s.sky_color)
        r.fill_poly(ground, s.ground_color)

        # --- Main horizon line ---
        r.line(p1, p2, s.line, self.thickness)



import time
# =========================
# HUD Orchestrator
# =========================
class Hud:
    """Creates and renders widgets according to a declarative layout."""
    def __init__(self, frame_size: Tuple[int, int],
                 layout_spec: Dict[str, Dict],
                 style: Optional[Style] = None,
                 stream_url=None):
        frame_w, frame_h = frame_size
        self.frame_w = max(frame_w, 1)
        self.frame_h = max(frame_h, 1)
        self.style = style or Style()
        self.widgets: List[BaseWidget] = []
        self.video = False
        self._stretch_video = False  # If false - cuts video, if true - stretches
        self.stream_url = stream_url
        self.cap = None
        self.open_stream(self.stream_url)
        self.video_mirror = False
        self.layout_spec = layout_spec
        self._small_hud_enabled = False
        self.old_time = time.time()
        self._build_from_layout(layout_spec)

    
    def close_stream(self):
        """Release owned resources like VideoCapture (if you want Hud to own it)."""
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass


    def open_stream(self, url):
        if not url:
            return
        self.cap = cv2.VideoCapture(url)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_h)
        
    def stream_video(self, url):
        if url is None:
            self.video = False
            print("Closing stream")
            self.close_stream()
            self._build_from_layout(self.layout_spec)
            return


        self.video = True
        self.open_stream(url)

        self._build_from_layout(self.layout_spec)
        
    def switch_small_hud(self, small_hud):
        print("Switch to small hud")
        self._small_hud_enabled = small_hud
        self._build_from_layout(self.layout_spec)
        

    def switch_stretch_video(self, stretch_video):
        print("switch stretch video", stretch_video)
        self._stretch_video = bool(stretch_video)

        # Try to update existing VideoBackground in-place
        updated = False
        for w in self.widgets:
            if isinstance(w, VideoBackground):
                w.set_stretch_video(self._stretch_video)
                updated = True

        # If there's no VideoBackground yet (e.g., small HUD or video off) – rebuild when appropriate
        if not updated and self.video and not self._small_hud_enabled:
            self._build_from_layout(self.layout_spec)



    def _build_from_layout(self, spec: Dict[str, Dict]):
        """Instantiate widgets based on the provided layout spec."""
        s = self.style
        # Example widgets from your current HUD:
        self.widgets.clear()
        if self.video and not self._small_hud_enabled:
            print(self._small_hud_enabled, "small hud")
            self.widgets.append(
                VideoBackground(
                    layout=LayoutBox(anchor=tuple(spec["horizon"]["anchor"]),
                                     offset=tuple(spec["horizon"]["offset"])),
                    style=s,
                    cap=self.cap,
                    mirror=self.video_mirror,
                    stretch_video=self._stretch_video
                )
            )
        else:
            print("Horizon")
            self.widgets.append(ArtificialHorizon(
                layout=LayoutBox(anchor=tuple(spec["horizon"]["anchor"]),
                                 offset=tuple(spec["horizon"]["offset"])),
                style=s,
                thickness=2,
                length_factor=2.2,
                center_tick_len=16,
            ))
        # Roll arc (top)
        self.widgets.append(RollArc(
            layout=LayoutBox(anchor=tuple(spec["roll_arc"]["anchor"]),
                             offset=tuple(spec["roll_arc"]["offset"])),
            style=s,
        ))

        # Pitch ladder (center)
        self.widgets.append(PitchLadder(
            layout=LayoutBox(anchor=tuple(spec["pitch_ledder"]["anchor"]),
                             offset=tuple(spec["pitch_ledder"]["offset"])),
            style=s
        ))

        # Center wings (center)
        self.widgets.append(CenterWings(
            layout=LayoutBox(anchor=tuple(spec["wings"]["anchor"]),
                             offset=tuple(spec["wings"]["offset"])),
            style=s
        ))

        # Center reticle (center)
        self.widgets.append(CenterReticle(
            layout=LayoutBox(anchor=tuple(spec["reticle"]["anchor"]),
                             offset=tuple(spec["reticle"]["offset"])),
            style=s
        ))


        # Prepare initial layout for the given frame size
        for w in self.widgets:
            w.layout_frame(self.frame_w, self.frame_h)

    def resize(self, frame_size: Tuple[int, int]):
        """Call when frame size changes."""
        frame_w, frame_h = frame_size
        self.frame_w = max(frame_w, 1)
        self.frame_h = max(frame_h, 1)
        for w in self.widgets:
            w.layout_frame(self.frame_w, self.frame_h)
        print(f"resize hud {self.frame_w} {self.frame_h}")

    def render(self, telem: TelemetryDTO):
        """Draw all widgets in proper z-order."""
        self.img = np.zeros((self.frame_h, self.frame_w, 3), dtype=np.uint8)
        r = CvRenderer(self.img)
        new_time = time.time()
        # print(new_time - self.old_time)
        self.old_time = new_time
        for w in self.widgets:
            w.draw(r, telem)
            


    def shrink_and_place(self, hud_img: np.ndarray, scale: float = 0.33,
                         pos: str = "top-right") -> np.ndarray:
        """Shrink HUD and place it on top of video background."""
        h, w = hud_img.shape[:2]

        ok, frame = self.cap.read()
        if not ok or frame is None:
            background = np.zeros_like(hud_img)
        else:
            if self._stretch_video:
                background = cv2.resize(frame, (w, h), interpolation=cv2.INTER_AREA)
            else:
                background = VideoBackground._resize_cover(frame, w, h)

        new_w, new_h = int(w * scale), int(h * scale)
        small = cv2.resize(hud_img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        if pos == "top-right":
            x0, y0 = w - new_w, 0
        elif pos == "top-left":
            x0, y0 = 0, 0
        elif pos == "bottom-right":
            x0, y0 = w - new_w, h - new_h
        elif pos == "bottom-left":
            x0, y0 = 0, h - new_h
        else:
            raise ValueError("Bad position")

        background[y0:y0+new_h, x0:x0+new_w] = small
        return background




# =========================
# Example layout spec
# =========================
HUD_LAYOUT = {
    # Main primitives
    "roll_arc": {"anchor": ("top", "center"), "offset": (0, "17%")},
    "pitch_ledder":    {"anchor": ("center", "center"), "offset": (0, 0)},
    "wings":    {"anchor": ("center", "center"), "offset": (0, 0)},
    "reticle":  {"anchor": ("center", "center"), "offset": (0, 0)},
    "horizon": {"anchor": ("center", "center"), "offset": (0, 0)},

}


if __name__ == "__main__":
    import time
    import math
    import cv2

    # Window name constant
    WIN = "HUD"

    # Create HUD with no video background (pure artificial horizon)
    hud = Hud((800, 600), HUD_LAYOUT, stream_url=None)

    # --- simple telemetry simulator ---
    # Returns TelemetryDTO with smoothly changing roll/pitch/yaw
    def simulate_telem(t: float) -> TelemetryDTO:
        roll  = 25.0 * math.sin(0.5 * t)
        pitch = 15.0 * math.sin(0.2 * t + 1.2)
        yaw   = (t * 12.0) % 360.0
        return TelemetryDTO(roll=roll, pitch=pitch, yaw=yaw, spd=22.5, alt=123.4)

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)

    prev_w, prev_h = hud.frame_w, hud.frame_h

    t0 = time.time()
    fps_timer = time.time()
    frames = 0

    while True:
        t = time.time() - t0

        telem = simulate_telem(t)  
        hud.render(telem)

        cv2.imshow(WIN, hud.img)

        # getWindowImageRect returns (x, y, w, h)
        x, y, w, h = cv2.getWindowImageRect(WIN)
        if w > 0 and h > 0 and (w != prev_w or h != prev_h):
            hud.resize((w, h))
            prev_w, prev_h = w, h

        if cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
            break

        key = cv2.waitKey(16) & 0xFF  # ~60 FPS target
        if key == ord('q'):
            break

        frames += 1
        now = time.time()
        if now - fps_timer >= 1.0:
            fps = frames / (now - fps_timer)
            print(f"[INFO] FPS: {fps:.1f}")
            fps_timer = now
            frames = 0

    cv2.destroyAllWindows()
