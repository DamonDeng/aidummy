"""
depth_driver.py — Phase 2 ctypes persistent driver for Orbbec Astra Pro

Loads libOrbbecSDK.dylib directly into Python via ctypes.
Keeps the device open between frames — no subprocess startup cost.
Provides the same interface as Phase 1 (DepthCamera) so camera_manager.py
can switch drivers transparently.

Usage:
    from depth_driver import DepthDriver
    driver = DepthDriver()
    driver.open()                          # open device once
    raw, W, H, scale = driver.capture()   # get raw uint16 depth array
    driver.close()                         # release device

    # Or as context manager:
    with DepthDriver() as driver:
        raw, W, H, scale = driver.capture()
"""
from __future__ import annotations

import ctypes
import ctypes.util
import os
import threading
import time
import struct
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

# ── SDK location ──────────────────────────────────────────────────────────────
SDK_DIR     = Path(__file__).parent / "orbbec_sdk"
SDK_DYLIB   = SDK_DIR / "libOrbbecSDK.dylib"

# ── OB constants (from ObTypes.h) ────────────────────────────────────────────
OB_STREAM_DEPTH   = 3    # OB_STREAM_DEPTH (IR=1, COLOR=2, DEPTH=3)
OB_FORMAT_Y11     = 11   # packed 11-bit, SDK auto-unpacks to uint16 mm
OB_LOG_OFF        = 5    # OB_LOG_SEVERITY_OFF (DEBUG=0,INFO=1,WARN=2,ERROR=3,FATAL=4,OFF=5)

# ── Opaque pointer types ──────────────────────────────────────────────────────
class _OBContext(ctypes.Structure):   pass
class _OBDeviceList(ctypes.Structure): pass
class _OBDevice(ctypes.Structure):    pass
class _OBPipeline(ctypes.Structure):  pass
class _OBConfig(ctypes.Structure):    pass
class _OBFrame(ctypes.Structure):     pass
class _OBError(ctypes.Structure):     pass

CtxP      = ctypes.POINTER(_OBContext)
DevListP  = ctypes.POINTER(_OBDeviceList)
DevP      = ctypes.POINTER(_OBDevice)
PipeP     = ctypes.POINTER(_OBPipeline)
CfgP      = ctypes.POINTER(_OBConfig)
FrameP    = ctypes.POINTER(_OBFrame)
ErrorP    = ctypes.POINTER(_OBError)
ErrorPP   = ctypes.POINTER(ErrorP)


def _load_lib() -> ctypes.CDLL:
    """Load libOrbbecSDK.dylib with its dependencies from orbbec_sdk/."""
    sdk_dir = str(SDK_DIR)
    # Prepend our SDK dir to DYLD_LIBRARY_PATH so dependencies resolve
    existing = os.environ.get("DYLD_LIBRARY_PATH", "")
    os.environ["DYLD_LIBRARY_PATH"] = f"{sdk_dir}:{existing}" if existing else sdk_dir
    return ctypes.CDLL(str(SDK_DYLIB))


def _bind(lib: ctypes.CDLL) -> None:
    """Declare argtypes/restype for all C API functions we use."""

    # --- Logger (global, call before creating context) ---
    lib.ob_set_logger_severity.argtypes = [ctypes.c_int, ErrorPP]
    lib.ob_set_logger_severity.restype  = None

    lib.ob_set_logger_to_console.argtypes = [ctypes.c_int, ErrorPP]
    lib.ob_set_logger_to_console.restype  = None

    # --- Context ---
    lib.ob_create_context.argtypes = [ErrorPP]
    lib.ob_create_context.restype  = CtxP

    lib.ob_delete_context.argtypes = [CtxP, ErrorPP]
    lib.ob_delete_context.restype  = None

    lib.ob_query_device_list.argtypes = [CtxP, ErrorPP]
    lib.ob_query_device_list.restype  = DevListP

    # --- Device list ---
    lib.ob_device_list_device_count.argtypes = [DevListP, ErrorPP]
    lib.ob_device_list_device_count.restype  = ctypes.c_uint32

    lib.ob_device_list_get_device.argtypes = [DevListP, ctypes.c_uint32, ErrorPP]
    lib.ob_device_list_get_device.restype  = DevP

    lib.ob_delete_device_list.argtypes = [DevListP, ErrorPP]
    lib.ob_delete_device_list.restype  = None

    lib.ob_delete_device.argtypes = [DevP, ErrorPP]
    lib.ob_delete_device.restype  = None

    # --- Pipeline ---
    lib.ob_create_pipeline_with_device.argtypes = [DevP, ErrorPP]
    lib.ob_create_pipeline_with_device.restype  = PipeP

    lib.ob_delete_pipeline.argtypes = [PipeP, ErrorPP]
    lib.ob_delete_pipeline.restype  = None

    lib.ob_pipeline_start_with_config.argtypes = [PipeP, CfgP, ErrorPP]
    lib.ob_pipeline_start_with_config.restype  = None

    lib.ob_pipeline_stop.argtypes = [PipeP, ErrorPP]
    lib.ob_pipeline_stop.restype  = None

    lib.ob_pipeline_wait_for_frameset.argtypes = [PipeP, ctypes.c_uint32, ErrorPP]
    lib.ob_pipeline_wait_for_frameset.restype  = FrameP

    # --- Config ---
    lib.ob_create_config.argtypes = [ErrorPP]
    lib.ob_create_config.restype  = CfgP

    lib.ob_delete_config.argtypes = [CfgP, ErrorPP]
    lib.ob_delete_config.restype  = None

    lib.ob_config_enable_video_stream.argtypes = [
        CfgP,
        ctypes.c_int,    # stream type
        ctypes.c_int,    # width
        ctypes.c_int,    # height
        ctypes.c_int,    # fps
        ctypes.c_int,    # format
        ErrorPP,
    ]
    lib.ob_config_enable_video_stream.restype = None

    # --- Frameset ---
    lib.ob_frameset_depth_frame.argtypes = [FrameP, ErrorPP]
    lib.ob_frameset_depth_frame.restype  = FrameP

    # --- Frame ---
    lib.ob_video_frame_width.argtypes  = [FrameP, ErrorPP]
    lib.ob_video_frame_width.restype   = ctypes.c_uint32

    lib.ob_video_frame_height.argtypes = [FrameP, ErrorPP]
    lib.ob_video_frame_height.restype  = ctypes.c_uint32

    lib.ob_frame_data.argtypes = [FrameP, ErrorPP]
    lib.ob_frame_data.restype  = ctypes.c_void_p

    lib.ob_frame_data_size.argtypes = [FrameP, ErrorPP]
    lib.ob_frame_data_size.restype  = ctypes.c_uint32

    lib.ob_depth_frame_get_value_scale.argtypes = [FrameP, ErrorPP]
    lib.ob_depth_frame_get_value_scale.restype  = ctypes.c_float

    lib.ob_delete_frame.argtypes = [FrameP, ErrorPP]
    lib.ob_delete_frame.restype  = None


class OrbbecError(RuntimeError):
    pass


class DepthDriver:
    """
    Persistent ctypes driver for Orbbec Astra Pro depth camera.

    Keeps the device and pipeline open between captures — no per-capture
    subprocess startup overhead. Suitable for frequent polling or streaming.

    Thread-safe: a mutex serializes captures so only one frame is in-flight
    at a time (the USB device doesn't support concurrent reads).
    """

    DEFAULT_W   = 640
    DEFAULT_H   = 480
    DEFAULT_FPS = 30
    WARMUP_FRAMES = 10

    def __init__(
        self,
        width:  int = DEFAULT_W,
        height: int = DEFAULT_H,
        fps:    int = DEFAULT_FPS,
    ) -> None:
        self.width  = width
        self.height = height
        self.fps    = fps
        self._lock  = threading.Lock()
        self._lib: Optional[ctypes.CDLL] = None
        self._ctx:  Optional[CtxP]  = None
        self._dev:  Optional[DevP]  = None
        self._pipe: Optional[PipeP] = None
        self._open = False

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def open(self) -> None:
        """Load SDK, open device, start depth pipeline."""
        with self._lock:
            if self._open:
                return
            self._lib = _load_lib()
            _bind(self._lib)
            err = ErrorP()
            ep  = ctypes.byref(err)

            # Suppress SDK console logs
            self._lib.ob_set_logger_severity(OB_LOG_OFF, ep)
            self._lib.ob_set_logger_to_console(OB_LOG_OFF, ep)

            # Create context + find device
            self._ctx = self._lib.ob_create_context(ep)
            self._check_error(err, "ob_create_context")

            dev_list = self._lib.ob_query_device_list(self._ctx, ep)
            self._check_error(err, "ob_query_device_list")

            count = self._lib.ob_device_list_device_count(dev_list, ep)
            self._check_error(err, "ob_device_list_device_count")
            if count == 0:
                self._lib.ob_delete_device_list(dev_list, ep)
                raise OrbbecError("No Orbbec device found")

            self._dev = self._lib.ob_device_list_get_device(dev_list, 0, ep)
            self._check_error(err, "ob_device_list_get_device")
            self._lib.ob_delete_device_list(dev_list, ep)

            # Create config: enable depth stream
            cfg = self._lib.ob_create_config(ep)
            self._check_error(err, "ob_create_config")
            self._lib.ob_config_enable_video_stream(
                cfg,
                OB_STREAM_DEPTH,
                self.width, self.height, self.fps,
                OB_FORMAT_Y11,
                ep,
            )
            self._check_error(err, "ob_config_enable_video_stream")

            # Create pipeline and start
            self._pipe = self._lib.ob_create_pipeline_with_device(self._dev, ep)
            self._check_error(err, "ob_create_pipeline_with_device")
            self._lib.ob_pipeline_start_with_config(self._pipe, cfg, ep)
            self._check_error(err, "ob_pipeline_start_with_config")
            self._lib.ob_delete_config(cfg, ep)

            # Warm up: discard first N frames
            self._warmup()
            self._open = True

    def close(self) -> None:
        """Stop pipeline and release all SDK resources."""
        with self._lock:
            if not self._open:
                return
            err = ErrorP()
            ep  = ctypes.byref(err)
            try:
                if self._pipe:
                    self._lib.ob_pipeline_stop(self._pipe, ep)
                    self._lib.ob_delete_pipeline(self._pipe, ep)
                    self._pipe = None
                if self._dev:
                    self._lib.ob_delete_device(self._dev, ep)
                    self._dev = None
                if self._ctx:
                    self._lib.ob_delete_context(self._ctx, ep)
                    self._ctx = None
            finally:
                self._open = False

    def __enter__(self) -> "DepthDriver":
        self.open()
        return self

    def __exit__(self, *_) -> None:
        self.close()

    # ── Capture ───────────────────────────────────────────────────────────────

    def capture(self) -> Tuple[np.ndarray, int, int, float]:
        """
        Capture one depth frame.

        Returns:
            raw   — numpy uint16 array, shape (H, W), values in depth units
            W     — frame width in pixels
            H     — frame height in pixels
            scale — mm per depth unit (usually 1.0, so values are already mm)

        Raises OrbbecError on failure.
        """
        if not self._open:
            raise OrbbecError("DepthDriver is not open — call open() first")

        with self._lock:
            err = ErrorP()
            ep  = ctypes.byref(err)

            frameset = None
            for _ in range(30):
                frameset = self._lib.ob_pipeline_wait_for_frameset(self._pipe, 500, ep)
                self._check_error(err, "ob_pipeline_wait_for_frameset")
                if frameset:
                    break
                time.sleep(0.05)

            if not frameset:
                raise OrbbecError("Timed out waiting for depth frameset")

            try:
                depth_frame = self._lib.ob_frameset_depth_frame(frameset, ep)
                self._check_error(err, "ob_frameset_depth_frame")
                if not depth_frame:
                    raise OrbbecError("No depth frame in frameset")

                try:
                    W     = self._lib.ob_video_frame_width(depth_frame, ep)
                    H     = self._lib.ob_video_frame_height(depth_frame, ep)
                    scale = self._lib.ob_depth_frame_get_value_scale(depth_frame, ep)
                    size  = self._lib.ob_frame_data_size(depth_frame, ep)
                    ptr   = self._lib.ob_frame_data(depth_frame, ep)

                    if not ptr:
                        raise OrbbecError("Null frame data pointer")

                    # Copy raw bytes out of SDK memory into numpy array
                    raw_bytes = (ctypes.c_uint8 * size).from_address(ptr)
                    raw = np.frombuffer(raw_bytes, dtype=np.uint16).reshape(H, W).copy()
                    return raw, int(W), int(H), float(scale)

                finally:
                    self._lib.ob_delete_frame(depth_frame, ep)
            finally:
                self._lib.ob_delete_frame(frameset, ep)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _warmup(self) -> None:
        """Discard the first N frames so auto-exposure stabilizes."""
        err = ErrorP()
        ep  = ctypes.byref(err)
        for _ in range(self.WARMUP_FRAMES):
            fs = self._lib.ob_pipeline_wait_for_frameset(self._pipe, 500, ep)
            if fs:
                self._lib.ob_delete_frame(fs, ep)

    @staticmethod
    def _check_error(err: ErrorP, fn_name: str = "") -> None:
        if err and err.contents:
            # Try to get error message from SDK
            try:
                lib = ctypes.CDLL(str(SDK_DYLIB))
                lib.ob_error_message.argtypes = [ErrorP]
                lib.ob_error_message.restype  = ctypes.c_char_p
                msg = lib.ob_error_message(err)
                msg_str = msg.decode() if msg else "unknown"
            except Exception:
                msg_str = "unknown"
            raise OrbbecError(f"SDK error in {fn_name}: {msg_str}")
