"""
camera_manager.py — Depth + RGB camera integration for robot server

Bridges the Orbbec Astra Pro depth camera (via depth_capture C++ binary)
and the RGB camera (via imagesnap) into a simple Python API.
"""
from __future__ import annotations

import subprocess
import tempfile
import json
import os
import threading
from pathlib import Path
from typing import Tuple

ROBOT_SERVER_DIR = Path(__file__).parent


class RGBCamera:
    """RGB camera via imagesnap (Astra Pro HD Camera / UVC)."""

    DEVICE    = "Astra Pro HD Camera"
    IMAGESNAP = "/opt/homebrew/bin/imagesnap"

    def capture(self, output_path: str, warmup_secs: float = 2.0) -> bool:
        """Capture a single RGB frame to output_path. Returns True on success."""
        result = subprocess.run(
            [self.IMAGESNAP, "-d", self.DEVICE, "-w", str(warmup_secs), output_path],
            capture_output=True,
            text=True,
            timeout=30,
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
            try:
                os.unlink(tmp)
            except OSError:
                pass


class DepthCamera:
    """Depth camera via depth_capture C++ binary (Orbbec Astra Pro, OpenNI protocol)."""

    BINARY  = str(ROBOT_SERVER_DIR / "depth_capture")
    SDK_DIR = str(ROBOT_SERVER_DIR / "orbbec_sdk")

    def _env(self) -> dict:
        """Build env with DYLD_LIBRARY_PATH set to orbbec_sdk/."""
        env = os.environ.copy()
        existing = env.get("DYLD_LIBRARY_PATH", "")
        env["DYLD_LIBRARY_PATH"] = f"{self.SDK_DIR}:{existing}" if existing else self.SDK_DIR
        return env

    def _check_binary(self) -> None:
        if not Path(self.BINARY).exists():
            raise FileNotFoundError(
                f"depth_capture binary not found at {self.BINARY}. "
                "Run: make depth_capture  (in the robot-server directory)"
            )

    def capture(
        self,
        output_path: str,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
    ) -> dict:
        """
        Run depth_capture binary, save PNG to output_path.
        Returns stats dict:
          {ok, width, height, valid_pixels, total_pixels, valid_pct,
           depth_min_mm, depth_max_mm, depth_p5_mm, depth_p95_mm,
           depth_mean_mm, output_path}
        Raises FileNotFoundError or RuntimeError on failure.
        """
        self._check_binary()
        result = subprocess.run(
            [self.BINARY, output_path, str(width), str(height), str(fps)],
            capture_output=True,
            text=True,
            timeout=60,
            env=self._env(),
        )
        if result.returncode != 0:
            raise RuntimeError(
                f"depth_capture exited {result.returncode}: {result.stderr.strip()}"
            )
        # SDK may print logs mixed into stdout — take the last non-empty line (JSON)
        lines = [l.strip() for l in result.stdout.splitlines() if l.strip()]
        if not lines:
            raise RuntimeError(f"depth_capture produced no output. stderr: {result.stderr.strip()}")
        json_line = lines[-1]
        try:
            stats = json.loads(json_line)
        except json.JSONDecodeError as e:
            raise RuntimeError(f"depth_capture output is not valid JSON: {json_line!r}") from e
        if not stats.get("ok"):
            raise RuntimeError(f"depth_capture reported error: {stats}")
        return stats

    def capture_bytes(
        self,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
    ) -> Tuple[bytes, dict]:
        """Capture depth frame, return (png_bytes, stats_dict)."""
        with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as f:
            tmp = f.name
        try:
            stats = self.capture(tmp, width, height, fps)
            return Path(tmp).read_bytes(), stats
        finally:
            try:
                os.unlink(tmp)
            except OSError:
                pass

    def capture_hires_bytes(self) -> Tuple[bytes, dict]:
        """Capture 1280×1024 @ 7fps depth frame (4× more detail, ~3s capture time)."""
        return self.capture_bytes(1280, 1024, 7)


class CameraManager:
    """Singleton manager for RGB + depth cameras."""

    _instance: CameraManager | None = None

    def __init__(self) -> None:
        self.rgb   = RGBCamera()
        self.depth = DepthCamera()

    @classmethod
    def get(cls) -> CameraManager:
        """Return shared CameraManager instance."""
        if cls._instance is None:
            cls._instance = CameraManager()
        return cls._instance
