from collections import deque

import json
import multiprocessing as mp
from queue import Empty
import threading
import time

import cv2
import numpy as np

from camera import worker
from geometry import aim, draw_axes, fit_samples, marker_pose, measure, number, point, rectangle
from models import DATA, REVISION
from robot import Robot
from vision import Detector, Tracker


class Service:
    def __init__(self):
        self.lock = threading.RLock()
        self.path = DATA / "settings.json"
        self.config = json.loads(self.path.read_text(encoding="utf8")) if self.path.exists() else {}
        self.robot = Robot()
        self.robot.address = self.config.get("address", "")
        self.tracker = Tracker()
        self.camera_status = "正在检测 RealSense D405"
        self.model_error = ""
        self.devices = []
        self.epoch = self.counter = 0
        self.history = deque(maxlen=5)
        self.frozen = self.profile_key = None
        self.frames = self.process = self.stop_camera = None
        self.done = threading.Event()
        self.poll_at = 0

    def save(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(self.config, ensure_ascii=False, allow_nan=False), encoding="utf8")
        tmp.replace(self.path)

    def profile(self):
        return self.config.setdefault("profiles", {}).setdefault(self.profile_key, {}) if self.profile_key else {}

    def change_camera(self, serial):
        self.robot.stop()
        self.reset()
        if self.process:
            self.stop_camera.set()
            self.process.join(.6)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(2)
            self.frames.close()
        self.config["d405_serial"] = serial
        ctx = mp.get_context("spawn")
        self.frames, self.stop_camera = ctx.Queue(1), ctx.Event()
        self.process = ctx.Process(target=worker, args=(serial, self.frames, self.stop_camera), daemon=True)
        self.process.start()
        self.camera_status = "正在检测 RealSense D405"

    def reset(self):
        self.epoch += 1
        self.profile_key = self.frozen = None
        self.history.clear()
        self.tracker = Tracker()

    def start(self):
        with self.lock:
            self.change_camera(self.config.get("d405_serial", ""))
        for target in (self.vision_loop, self.control_loop):
            threading.Thread(target=target, daemon=True).start()

    def close(self):
        self.done.set()
        with self.lock:
            try:
                self.robot.stop()
            finally:
                if self.stop_camera:
                    self.stop_camera.set()

    def vision_loop(self):
        try:
            cv2.setNumThreads(4)
            detector = Detector()
        except Exception as exc:
            detector = None
            self.model_error = str(exc)
        while not self.done.is_set():
            with self.lock:
                queue, epoch = self.frames, self.epoch
            try:
                f = queue.get(timeout=.25)
            except (Empty, ValueError, EOFError, OSError):
                continue
            try:
                with self.lock:
                    if epoch != self.epoch:
                        continue
                    if "devices" in f:
                        self.devices = f["devices"]
                    if "error" in f:
                        self.camera_status = f["error"]
                        self.reset()
                        continue
                    if "bgr" not in f or time.monotonic()-f["stamp"] > 1.2:
                        continue
                detections = detector.detect(f["bgr"], f["z"]) if detector else []
                with self.lock:
                    if epoch != self.epoch or time.monotonic()-f["stamp"] > 1.2:
                        continue
                    self.process_frame(f, detections)
            except Exception as exc:
                with self.lock:
                    self.camera_status = f"图像处理失败：{exc}"
                    self.reset()

    def process_frame(self, f, detections):
        robot_id = self.robot.state.get("robot")
        key = json.dumps([f["serial"], f["modes"], robot_id]) if robot_id else None
        if self.profile_key != key:
            self.robot.stop("设备或模式改变，请重新开启")
            self.reset()
            self.profile_key = key
        p = self.profile()
        people, selected, status = self.tracker.update(detections, f["stamp"], self.config.get("owner", []), p)
        angles, method, limit = aim(f, p, selected["uv"], selected["z"]) if selected else (None, "保持", False)
        self.counter += 1
        self.camera_status = "彩色与对齐深度" if f["z"] is not None else "彩色 · 深度不可用"
        self.history.append(dict(id=self.counter, frame=f, people=[t.copy() for t in people],
                                 selected=selected["id"] if selected else None, status=status,
                                 angles=angles, method=method, limit=limit))

    def latest(self):
        return self.history[-1] if self.history and time.monotonic()-self.history[-1]["frame"]["stamp"] <= 1.2 else None

    def control_loop(self):
        while not self.done.wait(.05):
            with self.lock:
                now = time.monotonic()
                try:
                    if self.robot.session:
                        snap = self.latest()
                        if snap is None or self.model_error:
                            self.robot.stop("相机或识别中断，请重新开启")
                        else:
                            self.robot.send(snap["angles"], now)
                    elif self.robot.address and now >= self.poll_at:
                        old = self.robot.state.get("robot")
                        self.poll_at = now + 2
                        state = self.robot.call()
                        if state.get("moving"):
                            self.poll_at = now + .25
                        if old != state["robot"]:
                            self.reset()
                            self.robot.reason = "已连接"
                except ValueError:
                    pass

    def snapshot(self, ident):
        for s in [self.frozen, *reversed(self.history)]:
            if s and s["id"] == ident:
                return s
        raise ValueError("画面已更新，请重新定格取点")

    def state(self):
        snap = self.latest()
        p = self.profile()
        return dict(camera=self.camera_status if snap or not self.history else "彩色帧已过期",
                    devices=self.devices, serial=self.config.get("d405_serial", ""), model_error=self.model_error,
                    address=self.robot.address, robot=self.robot.state, reason=self.robot.reason,
                    active=bool(self.robot.session), owner=len(self.config.get("owner", [])),
                    profile=bool(self.profile_key), roi=p.get("roi"), range=p.get("range"), trim=p.get("trim", [0,0]),
                    regions={k:dict(count=len(r["samples"]), rms=r.get("rms"), rect=r["rect"])
                             for k,r in p.get("regions", {}).items()}, pose=p.get("pose", {}).get("rms"),
                    checks=p.get("checks", [])[-6:], frame=snap["id"] if snap else None,
                    status=snap["status"] if snap else "等待画面", method=snap["method"] if snap else "保持",
                    angles=snap["angles"] if snap else None, limit=snap["limit"] if snap else False,
                    stream=({k:snap["frame"][k] for k in ("serial","model","modes","unit","device_stamp")} if snap else None))

    def image(self, ident, overlay):
        s = self.snapshot(ident)
        f = s["frame"]
        image = f["bgr"].copy()
        h, w = image.shape[:2]
        if overlay and f["z"] is not None:
            z = f["z"]
            col = cv2.applyColorMap(np.uint8(np.clip((z - .07) * (255 / (.50 - .07)), 0, 255)), cv2.COLORMAP_TURBO)
            mask = z > 0
            image[mask] = cv2.addWeighted(image, .48, col, .52, 0)[mask]
        p = self.profile()
        draw_axes(image, f, p.get("pose"))
        for check in p.get("checks", [])[-12:]:
            pos = tuple(np.rint(np.array(check["uv"])*[w-1,h-1]).astype(int))
            error = check.get("error")
            label = "--" if error is None else f"{error:+.2f}m"
            color = (40,180,245) if error is None else (220,205,85)
            cv2.drawMarker(image, pos, color, cv2.MARKER_CROSS, 12)
            cv2.putText(image, label, (pos[0]+7,pos[1]), 0, .45, color, 1)
        def box(rect, color, width):
            a,b,c,d = np.rint(np.array(rect)*[w-1,h-1,w-1,h-1]).astype(int)
            cv2.rectangle(image, (a,b), (c,d), color, width)
            return a,b
        box(p.get("roi", [0,0,1,1]), (185,180,50), 1)
        for t in s["people"]:
            selected = t["id"] == s["selected"]
            color = (35,155,255) if selected else (180,225,180)
            a,b = box(t["box"], color, 3 if selected else 1)
            role = "owner" if t["role"] == "主人" else "visitor" if t["visitor"] else "?"
            dist = f'{t["z"]:.2f}m' if t["z"] is not None else "--"
            cv2.putText(image, f'#{t["id"]} {role} {dist}', (a,max(18,b-5)), 0, .6, color, 2)
        return cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()

    def command(self, op, data):
        r = self.robot
        if op == "connect":
            r.connect(data["address"])
            self.config["address"] = r.address
            self.reset()
        elif op == "camera":
            self.change_camera(str(data.get("serial", "")))
        elif op == "start":
            if not self.latest() or self.model_error:
                raise ValueError(self.model_error or "请等待新鲜的相机画面")
            if not self.profile_key:
                raise ValueError("请先连接机器人并等待画面刷新")
            r.start()
            return
        elif op == "stop":
            r.stop()
            return
        elif op == "speed":
            speed = data.get("value")
            if type(speed) is not int or speed not in (5, 10, 15, 20):
                raise ValueError("请选择 5、10、15 或 20°/秒")
            if "manualSpeed" not in r.state:
                raise ValueError("请连接支持手动调速的固件")
            r.call("speed", value=speed)
            return
        elif op == "manual":
            state = r.call()
            if "x" in data or "y" in data:
                if set(data) != {"x", "y"}:
                    raise ValueError("请同时指定水平和垂直目标角")
                x, y = data["x"], data["y"]
                if type(x) is not int or type(y) is not int:
                    raise ValueError("目标角须为整数")
                if not (state["minX"] <= x <= state["maxX"] and state["minY"] <= y <= state["maxY"]):
                    raise ValueError("目标角超出舵机行程")
            elif data.get("center"):
                x, y = 90, 90
            else:
                x = np.clip(round(state["targetX"]) + int(number(data.get("dx", 0), -1, 1)), state["minX"], state["maxX"])
                y = np.clip(round(state["targetY"]) + int(number(data.get("dy", 0), -1, 1)), state["minY"], state["maxY"])
            r.manual(x, y)
            self.poll_at = time.monotonic() + .15
            return {"robot": r.state}
        elif op == "delete_owner":
            self.config["owner"] = []
            self.tracker = Tracker()
        elif op == "freeze":
            chosen = self.snapshot(data["frame"])
            self.frozen = dict(chosen, depth_history=[s for s in self.history if s["id"] <= chosen["id"]])
            return
        elif op == "enroll":
            snap = self.snapshot(data["frame"])
            uv = point(data["uv"])
            matches = [t for t in snap["people"] if t["box"][0]<=uv[0]<=t["box"][2] and t["box"][1]<=uv[1]<=t["box"][3]]
            if len(matches)!=1 or matches[0]["feature"] is None or matches[0]["ambiguous"]:
                raise ValueError("请选中单独、清晰的主人正脸或轻微侧脸，再录入")
            owner = self.config.setdefault("owner", [])
            feature = matches[0]["feature"].tolist()
            if feature not in owner:
                owner.append(feature)
                del owner[:-12]
            self.config["recognition"] = dict(revision=REVISION, owner_cosine=.40, other_cosine=.27)
            self.tracker = Tracker()
        elif op == "measure":
            z,valid,mad = measure(self.snapshot(data["frame"])["frame"]["z"], point(data["uv"]))
            return dict(m=z, valid=valid, mad=mad)
        else:
            if not self.profile_key:
                raise ValueError("请先连接相机和机器人")
            self.calibrate(op, data)
        self.save()

    def calibrate(self, op, data):
        p = self.profile()
        if op in ("roi","region","marker","sample","trim","range","clear_pose","clear_region"):
            self.robot.stop("标定设置已更改")
        if op == "roi":
            p["roi"] = rectangle(data["rect"])
        elif op == "range":
            values = data.get("range")
            if values:
                low, high = [number(v,.01,100) for v in values]
                if low >= high:
                    raise ValueError("近界须小于远界")
                values = [low,high]
            p["range"] = values
        elif op == "trim":
            p["trim"] = [number(v,-20,20) for v in data["trim"]]
        elif op == "clear_pose":
            p.pop("pose", None)
        elif op in ("region", "clear_region"):
            regions = p.setdefault("regions", {})
            name = str(data["name"]).strip()[:32]
            if not name:
                raise ValueError("请输入区域名称")
            if op == "clear_region":
                regions.pop(name, None)
            else:
                if name in regions:
                    raise ValueError("区域名称已存在，请使用新名称或先删除旧区域")
                regions[name] = dict(rect=rectangle(data["rect"]), samples=[])
        else:
            snap = self.snapshot(data["frame"])
            f = snap["frame"]
            if op == "marker":
                p["pose"] = marker_pose(f, data["options"])
                return
            uv = point(data["uv"])
            z, valid, mad = measure(f["z"], uv)
            if op == "sample":
                region = p.get("regions", {}).get(data["name"])
                if not region or not (region["rect"][0]<=uv[0]<=region["rect"][2] and region["rect"][1]<=uv[1]<=region["rect"][3]):
                    raise ValueError("请在指定二维区域内取点")
                st = self.robot.call()
                if st["mode"] != "manual" or st["moving"]:
                    raise ValueError("请手动对准并等待舵机到位")
                samples = region["samples"] + [[*uv, st["x"], st["y"]]]
                if any(np.linalg.norm(np.array(uv)-s[:2]) < .015 for s in region["samples"]):
                    raise ValueError("采样点重复，请选择新的位置")
                if len(samples) >= 5:
                    region["matrix"], region["rms"] = fit_samples(samples)
                region["samples"] = samples
            elif op == "check":
                truth = number(data["truth"], .01, 100)
                history = snap.get("depth_history", [snap])
                readings = [measure(item["frame"]["z"], uv)[0] for item in history]
                readings = [v for v in readings if v is not None]
                jitter = float(np.std(readings)) if len(readings) >= 2 else None
                p.setdefault("checks", []).append(dict(uv=uv, truth=truth, measured=z, valid=valid, mad=mad,
                                                        jitter=jitter, frames=len(readings),
                                                        error=None if z is None else round(z-truth, 3)))
                del p["checks"][:-50]
            elif op == "test":
                angles, _, _ = aim(f, p, uv, z)
                if angles is None:
                    raise ValueError("该位置缺少有效标定，云台保持")
                self.robot.manual(*angles)
            else:
                raise ValueError("未知操作")
