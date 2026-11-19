import argparse
import multiprocessing
from pathlib import Path

from flask import Flask, Response, abort, jsonify, request, send_from_directory
import cv2
from service import Service


def create_app(service):
    app = Flask(__name__, static_folder=None)
    app.config["MAX_CONTENT_LENGTH"] = 8192
    assets = Path(__file__).parent / "web"

    @app.before_request
    def local_only():
        if request.host.split(":")[0] not in {"127.0.0.1", "localhost"}:
            abort(403)
        if request.method == "POST" and request.headers.get("X-Desk-Control") != "1":
            abort(403)

    @app.get("/")
    def page():
        return send_from_directory(assets, "index.html")

    @app.get("/assets/<name>")
    def asset(name):
        return send_from_directory(assets, name)

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

    @app.after_request
    def no_cache(response):
        response.headers["Cache-Control"] = "no-store"
        response.headers["X-Frame-Options"] = "DENY"
        return response
    return app


if __name__ == "__main__":
    multiprocessing.freeze_support()
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    backend = Service()
    backend.start()
    try:
        create_app(backend).run(host="127.0.0.1", port=args.port, threaded=True, use_reloader=False)
    finally:
        backend.close()
