"""Acquire RealSense D405 color and metric depth in one rectified pixel grid."""
from queue import Empty, Full
import time

import cv2
import numpy as np
import pyrealsense2 as rs


WIDTH, HEIGHT, FPS = 640, 480, 30


def publish(queue, item):
    try:
        queue.put_nowait(item)
    except Full:
        try:
            queue.get(timeout=.05)
            queue.put_nowait(item)
        except (Empty, Full):
            pass


def select_camera(devices, serial):
    if not devices:
        raise ValueError("未发现 D405，请检查 USB 连接")
    if serial:
        for device in devices:
            if device["serial"] == serial:
                return device
        raise ValueError("所选 D405 未连接，请选择已连接的序列号")
    if len(devices) != 1:
        raise ValueError("检测到多台 D405，请选择相机序列号")
    return devices[0]


def color_geometry(intr):
    K = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1.]])
    if intr.model == rs.distortion.none or not any(intr.coeffs):
        return K, None
    # Map each ideal pinhole ray into the native color image once per stream.
    # The D405 can report Inverse Brown-Conrady, not OpenCV's distortion model.
    xs = [(u - intr.ppx) / intr.fx for u in range(intr.width)]
    pixels = np.empty((intr.height, intr.width, 2), np.float32)
    for v in range(intr.height):
        y = (v - intr.ppy) / intr.fy
        pixels[v] = [rs.rs2_project_point_to_pixel(intr, [x, y, 1.]) for x in xs]
    return K, pixels


def capture(serial, queue, stop):
    ctx = rs.context()
    devices = []
    for dev in ctx.query_devices():
        model = dev.get_info(rs.camera_info.name)
        if "D405" in model.split():
            devices.append(dict(serial=dev.get_info(rs.camera_info.serial_number), model=model))
    try:
        device = select_camera(devices, serial)
    except ValueError as exc:
        publish(queue, dict(devices=devices, error=str(exc)))
        return

    config = rs.config()
    config.enable_device(device["serial"])
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
    config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
    pipe = rs.pipeline(ctx)
    try:
        profile = pipe.start(config)
    except RuntimeError as exc:
        publish(queue, dict(devices=devices, error=f"D405 启动失败，请检查 USB 连接并关闭占用相机的程序。{exc}"))
        return
    try:
        unit = profile.get_device().first_depth_sensor().get_depth_scale()
        if not np.isfinite(unit) or unit <= 0:
            raise ValueError("D405 深度单位无效")
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().intrinsics
        K, pixels = color_geometry(intr)
        align = rs.align(rs.stream.color)
        modes = [WIDTH, HEIGHT, "bgr8-rectified", FPS, [WIDTH, HEIGHT, "z16-to-color"]]
        last_color = time.monotonic()
        prev_color = prev_depth = None
        while not stop.is_set():
            ok, frames = pipe.try_wait_for_frames(400)
            stamp = time.monotonic()
            color = frames.get_color_frame() if ok else None
            if not color or color.frame_number == prev_color:
                if stamp - last_color > 3:
                    raise RuntimeError("D405 彩色流中断，请检查 USB 连接")
                continue
            last_color, prev_color = stamp, color.frame_number
            native = np.asanyarray(color.get_data())
            # The multiprocessing queue serializes asynchronously. Own the arrays.
            bgr = cv2.remap(native, pixels, None, cv2.INTER_LINEAR) if pixels is not None else native.copy()
            depth = frames.get_depth_frame()
            z = None
            # The two streams have independent counters. Reject reused or stale depth.
            if (depth and depth.frame_number != prev_depth and
                    color.frame_timestamp_domain == depth.frame_timestamp_domain and
                    abs(color.timestamp - depth.timestamp) <= 500 / FPS):
                paired = align.process(frames).get_depth_frame()
                if paired:
                    raw = np.asanyarray(paired.get_data())
                    if raw.shape == bgr.shape[:2]:
                        if pixels is not None:
                            raw = cv2.remap(raw, pixels, None, cv2.INTER_NEAREST)
                        z = raw.astype(np.float32) * unit
                        prev_depth = depth.frame_number
            publish(queue, dict(**device, devices=devices, modes=modes, unit=unit,
                                bgr=bgr, z=z, K=K, dist=np.zeros(5), stamp=stamp,
                                device_stamp=color.timestamp))
    finally:
        pipe.stop()


def worker(serial, queue, stop):
    while not stop.is_set():
        try:
            capture(serial, queue, stop)
        except Exception as exc:
            publish(queue, {"error": str(exc)})
        stop.wait(2)
