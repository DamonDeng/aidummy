"""
camera_manager.py — Depth + RGB camera integration for robot server

Phase 1: subprocess bridge (DepthCamera) — one-shot captures via C++ binary
Phase 2: ctypes persistent driver (DepthDriver) — device stays open, ~30ms/frame

RGBCamera always uses imagesnap (UVC, no SDK needed).
CameraManager auto-promotes to Phase 2 driver on first use.
"""
from __future__ import annotations

import io
import os
import struct
import subprocess
import tempfile
import threading
import json
import time
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

ROBOT_SERVER_DIR = Path(__file__).parent


# ══════════════════════════════════════════════════════════════════════════════
# Colormap helpers (shared between Phase 1 C++ and Phase 2 numpy)
# ══════════════════════════════════════════════════════════════════════════════

def _red_to_blue_lut() -> np.ndarray:
    """Pre-compute 256-entry RGB look-up table: index 0=red, 255=blue."""
    lut = np.zeros((256, 3), dtype=np.uint8)
    for i in range(256):
        t = i / 255.0
        h = t * 240.0          # HSV hue: 0=red → 240=blue
        seg = int(h / 60.0)
        f   = h / 60.0 - seg
        q, u = 1.0 - f, f
        if   seg == 0: r,g,b = 1, u, 0
        elif seg == 1: r,g,b = q, 1, 0
        elif seg == 2: r,g,b = 0, 1, u
        elif seg == 3: r,g,b = 0, q, 1
        else:          r,g,b = 0, 0, 1
        lut[i] = [int(r*255), int(g*255), int(b*255)]
    return lut

_LUT = _red_to_blue_lut()


def depth_to_png(raw: np.ndarray, scale: float = 1.0) -> Tuple[bytes, dict]:
    """
    Convert raw uint16 depth array → colorized PNG bytes + stats dict.

    Color: red=close, blue=far (auto p5–p95 range). Dark gray = no data.
    Returns (png_bytes, stats).
    """
    import png  # pypng — lightweight, no OpenCV needed

    H, W = raw.shape
    valid_mask = raw > 0
    valid_vals = raw[valid_mask].astype(np.float32) * scale

    if len(valid_vals) == 0:
        raise RuntimeError("No valid depth pixels")

    valid_sorted = np.sort(valid_vals)
    n = len(valid_sorted)
    p5  = float(valid_sorted[n * 5  // 100])
    p95 = float(valid_sorted[n * 95 // 100])
    span = max(p95 - p5, 1.0)

    # Map depth → 0..255 index into LUT
    depth_mm = raw.astype(np.float32) * scale
    t = np.clip((depth_mm - p5) / span, 0.0, 1.0)
    idx = (t * 255).astype(np.uint8)

    # Build RGB image
    rgb = np.where(valid_mask[:, :, np.newaxis], _LUT[idx], np.array([30, 30, 30], dtype=np.uint8))

    # Encode as PNG via pypng
    rows = rgb.reshape(H, W * 3).tolist()
    buf = io.BytesIO()
    writer = png.Writer(width=W, height=H, bitdepth=8, greyscale=False)
    writer.write(buf, rows)
    png_bytes = buf.getvalue()

    stats = {
        "ok": True,
        "width": int(W),
        "height": int(H),
        "valid_pixels": int(n),
        "total_pixels": int(W * H),
        "valid_pct": round(n * 100.0 / (W * H), 1),
        "depth_min_mm": int(valid_vals.min()),
        "depth_max_mm": int(valid_vals.max()),
        "depth_p5_mm":  int(p5),
        "depth_p95_mm": int(p95),
        "depth_mean_mm": int(valid_vals.mean()),
    }
    return png_bytes, stats


# ══════════════════════════════════════════════════════════════════════════════
# RGB Camera (Phase 1 — imagesnap subprocess, UVC)
# ══════════════════════════════════════════════════════════════════════════════

class RGBCamera:
    """RGB camera via imagesnap (Astra Pro HD Camera / UVC)."""

    DEVICE    = "Astra Pro HD Camera"
    IMAGESNAP = "/opt/homebrew/bin/imagesnap"

    def capture(self, output_path: str, warmup_secs: float = 2.0) -> bool:
        """Capture a single RGB frame to output_path. Returns True on success."""
        result = subprocess.run(
            [self.IMAGESNAP, "-d", self.DEVICE, "-w", str(warmup_secs), output_path],
            capture_output=True, text=True, timeout=30,
        )
        return result.returncode == 0 and Path(output_path).exists()

    def capture_bytes(self, warmup_secs: float = 2.0) -> bytes:
        """Capture RGB frame and return JPEG bytes."""
        with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as f:
            tmp = f.name
        try:
            if not self.capture(tmp, warmup_secs):
                raise RuntimeError("imagesnap capture failed")
            return Path(tmp).read_bytes()
        finally:
            try: os.unlink(tmp)
            except OSError: pass


# ══════════════════════════════════════════════════════════════════════════════
# Depth Camera — Phase 1 (subprocess, fallback)
# ══════════════════════════════════════════════════════════════════════════════

class DepthCamera:
    """Depth camera via depth_capture C++ binary (fallback / one-shot use)."""

    BINARY  = str(ROBOT_SERVER_DIR / "depth_capture")
    SDK_DIR = str(ROBOT_SERVER_DIR / "orbbec_sdk")

    def _env(self) -> dict:
        env = os.environ.copy()
        existing = env.get("DYLD_LIBRARY_PATH", "")
        env["DYLD_LIBRARY_PATH"] = f"{self.SDK_DIR}:{existing}" if existing else self.SDK_DIR
        return env

    def _check_binary(self) -> None:
        if not Path(self.BINARY).exists():
            raise FileNotFoundError(
                f"depth_capture binary not found at {self.BINARY}. "
                "Run: make depth_capture"
            )

    def capture(self, output_path: str, width: int = 640, height: int = 480, fps: int = 30) -> dict:
        self._check_binary()
        result = subprocess.run(
            [self.BINARY, output_path, str(width), str(height), str(fps)],
            capture_output=True, text=True, timeout=60, env=self._env(),
        )
        if result.returncode != 0:
            raise RuntimeError(f"depth_capture failed: {result.stderr.strip()}")
        lines = [l.strip() for l in result.stdout.splitlines() if l.strip()]
        if not lines:
            raise RuntimeError(f"depth_capture produced no output")
        try:
            stats = json.loads(lines[-1])
        except json.JSONDecodeError as e:
            raise RuntimeError(f"depth_capture output not valid JSON: {lines[-1]!r}") from e
        if not stats.get("ok"):
            raise RuntimeError(f"depth_capture error: {stats}")
        return stats

    def capture_bytes(self, width: int = 640, height: int = 480, fps: int = 30) -> Tuple[bytes, dict]:
        with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as f:
            tmp = f.name
        try:
            stats = self.capture(tmp, width, height, fps)
            return Path(tmp).read_bytes(), stats
        finally:
            try: os.unlink(tmp)
            except OSError: pass

    def capture_hires_bytes(self) -> Tuple[bytes, dict]:
        return self.capture_bytes(1280, 1024, 7)


# ══════════════════════════════════════════════════════════════════════════════
# Depth Camera — Phase 2 (ctypes persistent driver)
# ══════════════════════════════════════════════════════════════════════════════

class PersistentDepthCamera:
    """
    Phase 2 depth camera: ctypes driver, device stays open.

    Open cost: ~0.5s (one-time).
    Capture cost: ~30ms per frame.
    Supports resolution switching via reopen().
    """

    DEFAULT_W   = 640
    DEFAULT_H   = 480
    DEFAULT_FPS = 30

    def __init__(self) -> None:
        self._driver: Optional[object] = None
        self._driver_lock = threading.Lock()
        self._width  = self.DEFAULT_W
        self._height = self.DEFAULT_H
        self._fps    = self.DEFAULT_FPS

    def _ensure_open(self, width: int, height: int, fps: int) -> "DepthDriver":
        from depth_driver import DepthDriver
        with self._driver_lock:
            need_reopen = (
                self._driver is None
                or self._width  != width
                or self._height != height
                or self._fps    != fps
            )
            if need_reopen:
                if self._driver is not None:
                    try: self._driver.close()
                    except Exception: pass
                drv = DepthDriver(width=width, height=height, fps=fps)
                drv.open()
                self._driver = drv
                self._width, self._height, self._fps = width, height, fps
        return self._driver

    def capture_raw(self, width: int = DEFAULT_W, height: int = DEFAULT_H, fps: int = DEFAULT_FPS):
        """Return (raw uint16 np.ndarray, W, H, scale_mm)."""
        driver = self._ensure_open(width, height, fps)
        return driver.capture()

    def capture_bytes(self, width: int = DEFAULT_W, height: int = DEFAULT_H, fps: int = DEFAULT_FPS) -> Tuple[bytes, dict]:
        """Capture depth frame → (PNG bytes, stats dict). Fast (~30ms after warmup)."""
        raw, W, H, scale = self.capture_raw(width, height, fps)
        return depth_to_png(raw, scale)

    def capture_hires_bytes(self) -> Tuple[bytes, dict]:
        return self.capture_bytes(1280, 1024, 7)

    def capture_stats(self, width: int = DEFAULT_W, height: int = DEFAULT_H, fps: int = DEFAULT_FPS) -> dict:
        """Return stats dict only (no PNG encoding overhead)."""
        raw, W, H, scale = self.capture_raw(width, height, fps)
        valid = raw[raw > 0].astype(np.float32) * scale
        if len(valid) == 0:
            return {"ok": False, "detail": "No valid depth pixels"}
        sv = np.sort(valid)
        n  = len(sv)
        return {
            "ok": True,
            "width": int(W), "height": int(H),
            "valid_pixels": int(n), "total_pixels": int(W * H),
            "valid_pct": round(n * 100.0 / (W * H), 1),
            "depth_min_mm":  int(valid.min()),
            "depth_max_mm":  int(valid.max()),
            "depth_p5_mm":   int(sv[n * 5  // 100]),
            "depth_p95_mm":  int(sv[n * 95 // 100]),
            "depth_mean_mm": int(valid.mean()),
        }

    def close(self) -> None:
        with self._driver_lock:
            if self._driver is not None:
                try: self._driver.close()
                except Exception: pass
                self._driver = None


# ══════════════════════════════════════════════════════════════════════════════
# CameraManager — singleton, auto-selects best available driver
# ══════════════════════════════════════════════════════════════════════════════

class CameraManager:
    """
    Singleton manager for all cameras.

    .rgb   → RGBCamera (imagesnap)
    .depth → PersistentDepthCamera (ctypes, ~30ms/frame)
    .depth_fallback → DepthCamera (subprocess, ~3s/frame, no numpy required)
    """

    _instance: Optional[CameraManager] = None

    def __init__(self) -> None:
        self.rgb              = RGBCamera()
        self.depth            = PersistentDepthCamera()
        self.depth_fallback   = DepthCamera()

    @classmethod
    def get(cls) -> CameraManager:
        if cls._instance is None:
            cls._instance = CameraManager()
        return cls._instance

    def close(self) -> None:
        """Release depth driver (call on server shutdown)."""
        self.depth.close()
