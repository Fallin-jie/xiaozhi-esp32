"""MV3D-RGBD capture worker. Only SDK-owned, paired frames are used for 3D."""
import ctypes as ct
from functools import lru_cache
import os
from pathlib import Path
from queue import Empty, Full
import sys
import time

import cv2
import numpy as np


@lru_cache(maxsize=1)
def load_sdk():
    program = Path(os.environ.get("ProgramFiles(x86)", "C:/Program Files (x86)"))
    roots = [Path(os.environ["MV3D_RGBD_SDK"])] if os.getenv("MV3D_RGBD_SDK") else []
    for base in (program, Path(os.environ.get("ProgramFiles", "C:/Program Files"))):
        for folder in ("HiViewer*", "Mv3dRgbd*", "Hikrobot*", "HIKROBOT*"):
            roots.extend(base.glob(folder))
    portable = Path(os.environ.get("LOCALAPPDATA", Path.home())) / "XiaozhiFollow" / "hik-sdk"
    roots.append(portable / "samples")
    for root in roots:
        module = next(root.rglob("Mv3dRgbdImport/Mv3dRgbdApi.py"), None)
        if module:
            sys.path.insert(0, str(module.parent.parent))
            break
    dll_dirs = os.environ.get("MV3D_RGBD_DLL_DIRS", "").split(os.pathsep)
    dll_dirs += [str(program / "Common Files" / name / "Runtime/Win64_x64")
                 for name in ("MV3D", "Mv3dRgbdSDK")]
    dll_dirs += [str(portable / name) for name in ("rgbd", "mv3d")]
    dll_dirs = [p for p in dll_dirs if p and Path(p).is_dir()]
    # Vendor plugins use LoadLibrary internally in addition to Python's DLL loader.
    os.environ["PATH"] = os.pathsep.join([*dll_dirs, os.environ.get("PATH", "")])
    handles = [os.add_dll_directory(p) for p in dll_dirs]
    try:
        from Mv3dRgbdImport import Mv3dRgbdDefine as d
        from Mv3dRgbdImport.Mv3dRgbdApi import Mv3dRgbd
    except (ImportError, OSError) as exc:
        raise RuntimeError(f"请安装 64 位 HiViewer 的 SDK、Python 样例和 USB 驱动。{exc}") from exc
    check(Mv3dRgbd.MV3D_RGBD_Initialize(), "初始化 SDK")
    return Mv3dRgbd, d, handles


def check(code, action):
    if code:
        raise RuntimeError(f"海康 {action} 失败 (0x{code:08x})")


def enumerate_devices(api, d):
    devices = d.MV3D_RGBD_DEVICE_INFO_LIST()
    count = ct.c_uint()
    check(api.MV3D_RGBD_GetDeviceNumber(d.DeviceType_USB, ct.byref(count)), "枚举")
    if count.value:
        check(api.MV3D_RGBD_GetDeviceList(d.DeviceType_USB, ct.pointer(devices.DeviceInfo[0]),
                                          len(devices.DeviceInfo), ct.byref(count)), "设备列表")
    def string(field):
        return bytes(field).split(b"\0")[0].decode(errors="replace")
    return [(info, string(info.chSerialNumber), string(info.chModelName))
            for info in devices.DeviceInfo[:count.value]]


def color_image(img, d):
    data = np.ctypeslib.as_array(img.pData, (img.nDataLen,))
    h, w, fmt = img.nHeight, img.nWidth, img.enImageType
    if fmt == d.ImageType_Rgbd:
        rgb = data.reshape(h, w, 5)[:, :, :3]
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    if fmt == d.ImageType_Jpeg:
        return cv2.imdecode(data, cv2.IMREAD_COLOR)
    if fmt == d.ImageType_RGB8_Planar:
        return np.ascontiguousarray(data.reshape(3, h, w)[::-1].transpose(1, 2, 0))
    if fmt == d.ImageType_YUV422:
        return cv2.cvtColor(data.reshape(h, w, 2), cv2.COLOR_YUV2BGR_YUY2)
    if fmt in (d.ImageType_YUV420SP_NV12, d.ImageType_YUV420SP_NV21):
        code = cv2.COLOR_YUV2BGR_NV12 if fmt == d.ImageType_YUV420SP_NV12 else cv2.COLOR_YUV2BGR_NV21
        return cv2.cvtColor(data.reshape(h * 3 // 2, w), code)
    return None


def publish(queue, item):
    try:
        queue.put_nowait(item)
    except Full:
        try:
            queue.get(timeout=.05)
            queue.put_nowait(item)
        except (Empty, Full):
            pass


def capture(serial, queue, stop):
    api, d, handles = load_sdk()
    records = enumerate_devices(api, d)
    names = [{"serial": sn, "model": model} for _, sn, model in records]
    if names:
        publish(queue, {"devices": names})
    candidates = [r for r in records if (serial and r[1] == serial) or
                  (not serial and "EB435" in r[2].upper())]
    if len(candidates) != 1:
        raise ValueError("请选择相机序列号" if candidates or names else "未发现相机，请连接 MV-EB435i 并关闭占用它的 HiViewer")
    info, sn, model = candidates[0]
    camera = api()
    check(camera.MV3D_RGBD_OpenDevice(ct.pointer(info)), "打开设备")
    started = False
    try:
        param = d.MV3D_RGBD_PARAM()
        param.enParamType = d.ParamType_Int
        param.ParamInfo.stIntParam.nCurValue = 1
        aligned = camera.MV3D_RGBD_SetParam(d.MV3D_RGBD_INT_IMAGEALIGN, ct.pointer(param)) == 0
        unit_param = d.MV3D_RGBD_PARAM()
        has_unit = camera.MV3D_RGBD_GetParam(d.MV3D_RGBD_FLOAT_Z_UNIT, ct.pointer(unit_param)) == 0
        unit = float(unit_param.ParamInfo.stFloatParam.fCurValue) / 1000 if has_unit else 0
        calib = d.MV3D_RGBD_CAMERA_PARAM()
        has_calib = camera.MV3D_RGBD_GetCameraParam(ct.pointer(calib)) == 0
        rgb = calib.stRgbCalibInfo
        previous = None
        check(camera.MV3D_RGBD_Start(), "开始采集")
        started = True
        last_rgb = time.monotonic()
        while not stop.is_set():
            if time.monotonic() - last_rgb > 3:
                raise RuntimeError("彩色流中断，请检查 USB 3.0 连接及彩色输出模式")
            frame = d.MV3D_RGBD_FRAME_DATA()
            if camera.MV3D_RGBD_FetchFrame(ct.pointer(frame), 400) or frame.nValidInfo:
                continue
            stamp = time.monotonic()
            color = depth = packed = None
            for img in frame.stImageData[:frame.nImageCount]:
                if img.enImageType == d.ImageType_Rgbd:
                    packed = img
                elif img.enImageType == d.ImageType_Depth:
                    depth = img
                elif img.enStreamType in (d.StreamType_Color, d.StreamType_Undefined) and img.enImageType in (
                    d.ImageType_Jpeg, d.ImageType_RGB8_Planar, d.ImageType_YUV422,
                    d.ImageType_YUV420SP_NV12, d.ImageType_YUV420SP_NV21):
                    color = img
            if color is None:
                color = packed
            if color is None or (color.nFrameNum, color.nTimeStamp) == previous:
                continue
            previous = (color.nFrameNum, color.nTimeStamp)
            bgr = color_image(color, d)
            if bgr is None:
                continue
            last_rgb = stamp
            h, w = bgr.shape[:2]
            K = dist = None
            if has_calib and (w, h) == (rgb.nWidth, rgb.nHeight):
                K = np.array(rgb.stIntrinsic.fData, dtype=float).reshape(3, 3)
                dist = np.zeros(8) if color.bIsRectified else np.array(rgb.stDistortion.fData, dtype=float)
                if not np.isfinite(K).all() or min(K[0, 0], K[1, 1]) <= 0:
                    K = None
            z = None
            depth_mode = None
            if (color.enImageType == d.ImageType_Rgbd and 0 < unit < 1 and
                    (color.enCoordinateType == d.CoordinateType_RGB or
                     (aligned and color.enCoordinateType == d.CoordinateType_Undefined))):
                data = np.ctypeslib.as_array(color.pData, (color.nDataLen,)).reshape(h, w, 5)
                raw = data[:, :, 3].astype(np.uint16) | (data[:, :, 4].astype(np.uint16) << 8)
                z = raw.astype(np.float32) * unit
                z[raw == 65535] = 0
                depth_mode = [w, h, color.enImageType, color.enCoordinateType]
            if depth is not None:
                depth_mode = [depth.nWidth, depth.nHeight, depth.enImageType, depth.enCoordinateType]
                paired = (depth.nFrameNum == color.nFrameNum or
                          (depth.nTimeStamp and depth.nTimeStamp == color.nTimeStamp))
                if paired and has_calib and unit > 0 and depth.enCoordinateType == d.CoordinateType_Depth:
                    converted = d.MV3D_RGBD_IMAGE_DATA()
                    ret = camera.MV3D_RGBD_ImageCoordinateTrans(ct.pointer(depth), unit * 1000,
                                                               ct.pointer(converted), ct.pointer(calib))
                    if ret == 0:
                        depth = converted
                if (0 < unit < 1 and (depth.enCoordinateType == d.CoordinateType_RGB or
                                      (aligned and depth.enCoordinateType == d.CoordinateType_Undefined))
                        and (depth.nWidth, depth.nHeight) == (w, h) and depth.nDataLen == w * h * 2
                        and paired):
                    raw = np.ctypeslib.as_array(depth.pData, (depth.nDataLen,)).view("<u2").reshape(h, w)
                    z = raw.astype(np.float32) * unit
                    z[raw == 65535] = 0
            publish(queue, dict(bgr=bgr, z=z, K=K, dist=dist, stamp=stamp, serial=sn, model=model,
                                modes=[w, h, color.enImageType, color.bIsRectified, depth_mode],
                                device_stamp=color.nTimeStamp, unit=unit, devices=names))
    finally:
        if started:
            camera.MV3D_RGBD_Stop()
        camera.MV3D_RGBD_CloseDevice()


def worker(serial, queue, stop):
    from models import DATA
    DATA.mkdir(parents=True, exist_ok=True)
    os.chdir(DATA)  # Native SDK writes diagnostic files relative to the worker's directory.
    while not stop.is_set():
        try:
            capture(serial, queue, stop)
        except Exception as exc:
            packet = {"error": str(exc)}
            if "未发现相机" in str(exc):
                packet["devices"] = []
            publish(queue, packet)
        stop.wait(2)
