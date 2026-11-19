import ipaddress
import json
import secrets
import time
from urllib.error import HTTPError
from urllib.request import Request, ProxyHandler, build_opener


class Robot:
    """Accessed under the service lock. A network failure always disarms local tracking."""
    def __init__(self):
        self.address = ""
        self.state = {}
        self.session = None
        self.reason = "待连接"
        self.last = None
        self.sent = 0
        self.http = build_opener(ProxyHandler({}))

    def call(self, op=None, **values):
        if not self.address:
            raise ValueError("请先连接机器人")
        data = json.dumps(dict(op=op, **values)).encode() if op else None
        url = "http://" + self.address + ("/api/follow" if op else "/api/state")
        req = Request(url, data, {"X-Desk-Control":"1", "Content-Type":"application/json", "Connection":"close"})
        try:
            with self.http.open(req, timeout=.85) as response:
                result = json.load(response)
        except HTTPError as exc:
            self.session = None
            self.reason = "控制权已切换，请重新开启跟随" if exc.code == 409 else f"固件返回 HTTP {exc.code}"
            raise ValueError(self.reason) from exc
        except (ValueError, OSError) as exc:
            self.session = None
            self.reason = "机器人离线，请检查地址和 Wi-Fi"
            self.state = {}
            raise ValueError(self.reason) from exc
        if op != "start":
            if "robot" not in result or "mode" not in result:
                raise ValueError("请先给机器人写入支持视觉跟随的新固件")
            self.state = result
        return result

    def connect(self, address):
        address = str(ipaddress.IPv4Address(address.strip()))
        self.stop()
        self.address, self.state = address, {}
        self.call()
        self.reason = "已连接"

    def start(self):
        if self.session:
            return
        expected = self.state.get("robot")
        state = self.call()
        if state["robot"] != expected:
            raise ValueError("机器人已更换，请重新连接并检查标定")
        args = dict(boot=state["boot"], generation=state["generation"], request=1+secrets.randbelow(2**32-1))
        self.session = self.call("start", **args)["session"]
        self.seq, self.sent, self.last = 0, 0, None
        self.reason = "跟随中"

    def send(self, angles, now):
        if not self.session or now-self.sent < .2:
            return
        new = tuple(angles) if angles else "hold"
        payload = {}
        if new != self.last:
            payload = dict(x=new[0], y=new[1]) if angles else dict(hold=True)
        elif now-self.sent < 1:
            return
        self.seq += 1
        self.call("update", session=self.session, seq=self.seq, **payload)
        self.last, self.sent = new, now

    def stop(self, reason="待开启"):
        session, self.session = self.session, None
        if session:
            self.call("stop", session=session)
        self.reason = reason

    def manual(self, x, y):
        self.call("manual", x=int(x), y=int(y))
        self.session, self.reason = None, "手动控制"
