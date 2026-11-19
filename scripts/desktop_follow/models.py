"""Download pinned OpenCV Zoo models into the user's local data directory."""
import hashlib
import os
import shutil
import tempfile
from pathlib import Path
from urllib.request import urlopen

DATA = Path(os.environ.get("XIAOZHI_FOLLOW_DATA", Path(os.environ.get("LOCALAPPDATA", Path.home())) / "XiaozhiFollow"))
REVISION = "48b10fa505aeb51fbd127fc60a5c660040e823b7"
MODELS = {
    "body": ("object_detection_yolox/object_detection_yolox_2022nov.onnx", "c5c2d13e59ae883e6af3b45daea64af4833a4951c92d116ec270d9ddbe998063"),
    "face": ("face_detection_yunet/face_detection_yunet_2023mar.onnx", "8f2383e4dd3cfbb4553ea8718107fc0423210dc964f9f4280604804ed2552fa4"),
    "identity": ("face_recognition_sface/face_recognition_sface_2021dec.onnx", "0ba9fbfa01b5270c96627c4ef784da859931e02f04419c829e83484087c34e79"),
}


def paths(download=False):
    directory = DATA / "models"
    directory.mkdir(parents=True, exist_ok=True)
    result = {}
    for name, (relative, digest) in MODELS.items():
        path = directory / Path(relative).name
        if not path.exists() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            if not download:
                raise ValueError("识别模型尚未就绪，请运行 python models.py")
            url = f"https://media.githubusercontent.com/media/opencv/opencv_zoo/{REVISION}/models/{relative}"
            with tempfile.NamedTemporaryFile(dir=directory, delete=False) as stream:
                tmp = Path(stream.name)
            try:
                with urlopen(url, timeout=90) as source, tmp.open("wb") as out:
                    shutil.copyfileobj(source, out)
                if hashlib.sha256(tmp.read_bytes()).hexdigest() != digest:
                    raise ValueError(f"模型校验失败 {path.name}")
                tmp.replace(path)
            finally:
                tmp.unlink(missing_ok=True)
        result[name] = str(path)
    return result


if __name__ == "__main__":
    print(paths(download=True))
