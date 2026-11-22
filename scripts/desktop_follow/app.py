import argparse
import json
import multiprocessing
from contextlib import ExitStack
from http.client import HTTPConnection, HTTPException
from ipaddress import IPv4Address, IPv4Network
from pathlib import Path
from threading import Thread

from flask import Flask, Response, abort, jsonify, redirect, request, send_from_directory
from werkzeug.serving import make_server
import cv2
from models import DATA
from service import Service


PRIVATE_NETWORKS = tuple(map(IPv4Network, ("10.0.0.0/8", "172.16.0.0/12", "192.168.0.0/16")))


def create_app(service, lan_ip=None, *, chat_only=False):
    app = Flask(__name__, static_folder=None)
    app.config["MAX_CONTENT_LENGTH"] = 8192
    assets = Path(__file__).parent / "web"

    @app.before_request
    def check_access():
        host = request.host.partition(":")[0]
        peer = IPv4Address(request.remote_addr)
        if host in {"127.0.0.1", "localhost"}:
            allowed = peer.is_loopback
        else:
            allowed = host == lan_ip and any(peer in network for network in PRIVATE_NETWORKS)
        if not allowed:
            abort(403)
        if request.method == "POST" and request.headers.get("X-Desk-Control") != "1":
            abort(403)

    @app.after_request
    def no_cache(response):
        response.headers["Cache-Control"] = "no-store"
        response.headers["X-Frame-Options"] = "DENY"
        return response

    @app.get("/")
    def page():
        if chat_only:
            # Host was validated above; return to the main visual console.
            return redirect(f"http://{request.host.partition(':')[0]}:8765/")
        return send_from_directory(assets, "index.html")

    @app.get("/debug")
    def debug_page():
        return send_from_directory(assets, "debug.html")

    @app.route("/api/debug/chat", methods=["GET", "POST"])
    def debug_chat():
        # Conversation history has its own size limit. Robot commands keep their 8 KiB limit.
        payload = None
        if request.method == "POST":
            request.max_content_length = 96 * 1024
            if not request.is_json:
                return jsonify(error="请发送 JSON 格式的对话。"), 415
            payload = request.get_data()
        upstream = HTTPConnection("127.0.0.1", 18765, timeout=85)
        try:
            upstream.request(request.method, "/chat", body=payload, headers={"Content-Type": "application/json", "X-Desk-Control": "1"})
            response = upstream.getresponse()
            data = json.loads(response.read(192 * 1024))
            fields = ("error",) if response.status >= 400 else ("model", "reply")
            return jsonify({k: data[k] for k in fields if k in data}), response.status
        except TimeoutError:
            return jsonify(error="模型响应超时，请重试。"), 504
        except (OSError, HTTPException, ValueError):
            return jsonify(error="文字调试服务未就绪，请检查本机的 chat-debug 容器。"), 503
        finally:
            upstream.close()

    @app.get("/assets/<name>")
    def asset(name):
        return send_from_directory(assets, name)

    if chat_only:
        return app

    @app.get("/api/state")
    def state():
        with service.lock:
            return jsonify(service.state())

    @app.get("/api/frame/<int:ident>")
    def frame(ident):
        try:
            with service.lock:
                data = service.image(ident, request.args.get("depth") == "1")
            return Response(data, mimetype="image/jpeg")
        except ValueError:
            abort(410)

    @app.get("/marker/<int:ident>.png")
    def marker(ident):
        if ident >= 50:
            abort(404)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        square = cv2.aruco.generateImageMarker(dictionary, ident, 600)
        bordered = cv2.copyMakeBorder(square, 80,80,80,80, cv2.BORDER_CONSTANT, value=255)
        return Response(cv2.imencode(".png", bordered)[1].tobytes(), mimetype="image/png")

    @app.post("/api/<op>")
    def command(op):
        data = request.get_json()
        if not isinstance(data, dict):
            abort(400)
        try:
            with service.lock:
                result = service.command(op, data)
            return jsonify(result or {"ok": True})
        except (ValueError, TypeError, KeyError, IndexError, cv2.error, OSError) as exc:
            return jsonify(error=str(exc)), 400

    return app


if __name__ == "__main__":
    multiprocessing.freeze_support()
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--chat-only", action="store_true", help="Serve text debugging without opening the camera")
    args = parser.parse_args()
    lan_config = DATA / "lan-ip.txt"
    lan_ip = None
    if lan_config.exists():
        try:
            address = IPv4Address(lan_config.read_text(encoding="utf-8-sig").strip())
            if not any(address in network for network in PRIVATE_NETWORKS):
                raise ValueError("expected a private LAN IPv4 address")
        except ValueError as exc:
            print(f"Check {lan_config}: {exc}", flush=True)
        else:
            lan_ip = str(address)

    backend = None if args.chat_only else Service()
    app = create_app(backend, lan_ip, chat_only=args.chat_only)
    with ExitStack() as servers:
        local = servers.enter_context(make_server("127.0.0.1", args.port, app, threaded=True))
        print(f"Local console: http://127.0.0.1:{args.port}/", flush=True)
        if lan_ip:
            try:
                lan = make_server(lan_ip, args.port, app, threaded=True)
            except SystemExit:  # Werkzeug exits on bind errors, e.g. after a DHCP change.
                print(f"LAN listener unavailable. Check {lan_config} and restart.", flush=True)
            else:
                servers.enter_context(lan)
                Thread(target=lan.serve_forever, daemon=True).start()
                servers.callback(lan.shutdown)
                print(f"LAN console: http://{lan_ip}:{args.port}/", flush=True)
        if backend is not None:
            servers.callback(backend.close)
            backend.start()
        local.serve_forever()
