import math

import cv2
import numpy as np


def in_rect(point, rect):
    x, y = point
    return rect[0] <= x <= rect[2] and rect[1] <= y <= rect[3]


def number(value, low, high):
    n = float(value)
    if not low <= n <= high:
        raise ValueError(f"数值须在 {low} 到 {high} 之间")
    return n


def point(values):
    if len(values) != 2:
        raise ValueError("请在图像中取点")
    return [number(v, 0, 1) for v in values]


def rectangle(values):
    a, b = point(values[:2]), point(values[2:])
    if min(b[0] - a[0], b[1] - a[1]) < .025:
        raise ValueError("框选区域太小")
    return a + b


def measure(z, uv, size=(.015, .015)):
    if z is None:
        return None, 0., None
    h, w = z.shape
    x = min(w - 1, round(uv[0] * (w - 1)))
    y = min(h - 1, round(uv[1] * (h - 1)))
    rx, ry = max(2, int(w * size[0])), max(2, int(h * size[1]))
    patch = z[max(0, y-ry):y+ry+1, max(0, x-rx):x+rx+1]
    good = patch[(patch > 0) & np.isfinite(patch)]
    ratio = good.size / patch.size
    if ratio < .35:
        return None, ratio, None
    median = float(np.median(good))
    mad = float(np.median(np.abs(good - median)))
    return (median if mad < max(.08, median * .12) else None), ratio, mad


def fit_samples(samples):
    if len(samples) < 5:
        raise ValueError("至少采集中、左、右、上、下五点")
    uv = np.array([s[:2] for s in samples])
    if min(np.linalg.norm(a - b) for i, a in enumerate(uv) for b in uv[:i]) < .015:
        raise ValueError("标定点过于接近，请重新取点")
    A = np.column_stack((uv, np.ones(len(uv))))
    if np.linalg.cond(A) > 150:
        raise ValueError("采样点接近共线，请增加上下和左右间距")
    angles = np.array([s[2:] for s in samples])
    matrix = np.linalg.lstsq(A, angles, rcond=None)[0]
    rms = float(np.sqrt(np.mean((A @ matrix - angles) ** 2)))
    if rms > 3:
        raise ValueError(f"拟合误差 {rms:.1f}°，请检查采样点")
    return matrix.tolist(), rms


def marker_pose(frame, options):
    if frame["K"] is None:
        raise ValueError("当前彩色模式缺少匹配的相机内参")
    size = number(options["size"], 10, 1000) / 1000
    marker_id = int(number(options["marker"], 0, 49))
    offsets = [number(options[k], -2000, 2000) / 1000 for k in ("right", "forward", "up")]
    yaw = math.radians(number(options["yaw"], -180, 180))
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    corners, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(frame["bgr"])
    if ids is None or list(ids.ravel()).count(marker_id) != 1:
        raise ValueError("画面中需要有且仅有一个指定编号的完整标记")
    image = corners[list(ids.ravel()).index(marker_id)].reshape(4, 2)
    h = size / 2
    obj = np.float64([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]])
    ok, rvec, tvec = cv2.solvePnP(obj, image, frame["K"], frame["dist"], flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok or tvec[2, 0] <= 0:
        raise ValueError("标记姿态无效")
    projection = cv2.projectPoints(obj, rvec, tvec, frame["K"], frame["dist"])[0].reshape(4, 2)
    rms = float(np.sqrt(np.mean(np.sum((projection - image) ** 2, axis=1))))
    if rms > 2.5:
        raise ValueError(f"重投影误差 {rms:.1f} px，请改善标记的大小或清晰度")
    # Flat marker's printed top = +Y, right = +X, outward normal = up.
    c, s = math.cos(yaw), math.sin(yaw)
    axes = cv2.Rodrigues(rvec)[0] @ np.array([[c,s,0],[-s,c,0],[0,0,1]])
    pivot = tvec.ravel() + axes @ offsets
    return dict(axes=axes.tolist(), pivot=pivot.tolist(), rms=rms, options=options)


def draw_axes(image, frame, pose, mirror=False):
    if not pose or frame["K"] is None:
        return
    axes, origin = np.array(pose["axes"]), np.array(pose["pivot"])
    length = float(pose["options"]["size"]) / 1000
    xyz = np.vstack((origin, origin + axes.T * length))
    if np.min(xyz[:, 2]) <= .01:
        return
    uv = cv2.projectPoints(xyz, np.zeros(3), np.zeros(3), frame["K"], frame["dist"])[0].astype(int).reshape(4, 2)
    if mirror:
        uv[:, 0] = image.shape[1] - 1 - uv[:, 0]
    for p, label, color in zip(uv[1:], ("R", "F", "U"), ((60,100,255),(80,235,150),(255,175,65))):
        cv2.arrowedLine(image, uv[0], p, color, 2)
        cv2.putText(image, label, p, 0, .65, color, 2)


def aim(frame, profile, uv, depth, *, camera_direction=False, robot=None):
    robot = robot or {}
    pose = profile.get("pose")
    trim = profile.get("trim", [0, 0])
    if pose and depth is not None and frame["K"] is not None:
        h, w = frame["bgr"].shape[:2]
        pixel = np.float64([[[uv[0] * (w-1), uv[1] * (h-1)]]])
        ray = cv2.undistortPoints(pixel, frame["K"], frame["dist"]).ravel()
        xyz = np.append(ray, 1.) * depth
        right, forward, up = np.array(pose["axes"]).T @ (xyz - pose["pivot"])
        angles = [90 + math.degrees(math.atan2(right, forward)) + trim[0],
                  90 - math.degrees(math.atan2(up, math.hypot(right, forward))) + trim[1]]
        method = "三维"
    else:
        region = next((r for r in reversed(list(profile.get("regions", {}).values()))
                       if r.get("matrix") and in_rect(uv, r["rect"])), None)
        if region:
            angles = np.array([*uv, 1.]) @ region["matrix"]
            angles -= [robot.get("centerOffsetX", 0), robot.get("centerOffsetY", 0)]
            method = "二维"
        elif camera_direction and not pose and not profile.get("regions") and frame["K"] is not None:
            # The D405 frames are rectified. Assume a fixed camera facing the same way as the centered head.
            h, w = frame["bgr"].shape[:2]
            K = frame["K"]
            right = (uv[0] * (w-1) - K[0, 2]) / K[0, 0]
            down = (uv[1] * (h-1) - K[1, 2]) / K[1, 1]
            # Horizontal commands increase toward the right of the mirrored preview.
            angles = [90 - math.degrees(math.atan(right)),
                      90 + math.degrees(math.atan2(down, math.hypot(1, right)))]
            method = "画面方向（估算）"
        else:
            return None, "保持", False
    bounds = np.clip(angles, [robot.get("minX",50), robot.get("minY",70)],
                             [robot.get("maxX",130), robot.get("maxY",110)])
    return [int(round(a)) for a in bounds], method, bool(np.any(bounds != angles))
