import cv2
import numpy as np

from geometry import in_rect, measure
from models import paths


def overlap(a, b):
    w = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    h = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    area = w * h
    return area / max(1e-8, (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - area)


class Detector:
    def __init__(self):
        files = paths()
        self.body = cv2.dnn.readNet(files["body"])
        self.face = cv2.FaceDetectorYN.create(files["face"], "", (320, 320), .85)
        self.recognizer = cv2.FaceRecognizerSF.create(files["identity"], "")
        grids, strides = [], []
        for stride in (8, 16, 32):
            y, x = np.mgrid[:640//stride, :640//stride]
            grids.extend(np.stack((x, y), -1).reshape(-1, 2))
            strides.extend([stride] * x.size)
        self.grid = np.array(grids)
        self.stride = np.array(strides)[:, None]

    def bodies(self, image):
        h, w = image.shape[:2]
        scale = 640 / max(h, w)
        small = cv2.resize(image, (max(1, int(w*scale)), max(1, int(h*scale))))
        padded = np.full((640, 640, 3), 114, np.uint8)
        padded[:small.shape[0], :small.shape[1]] = small
        self.body.setInput(cv2.dnn.blobFromImage(padded, swapRB=True))
        p = self.body.forward().reshape(-1, 85)
        scores = p[:, 4] * p[:, 5]
        mask = (scores > .4) & (np.argmax(p[:, 5:], axis=1) == 0)
        strides = self.stride[mask]
        centers = (p[mask, :2] + self.grid[mask]) * strides
        sizes = np.exp(p[mask, 2:4]) * strides
        boxes = np.column_stack((centers - sizes/2, sizes)) / scale
        keep = cv2.dnn.NMSBoxes(boxes.tolist(), scores[mask].tolist(), .4, .45)
        return [np.clip([x/w, y/h, (x+bw)/w, (y+bh)/h], 0, 1).tolist()
                for x, y, bw, bh in boxes[np.asarray(keep, dtype=int).ravel()]]

    def detect(self, image, z):
        h, w = image.shape[:2]
        boxes = self.bodies(image)
        scale = min(1, 800 / max(h, w))
        small = cv2.resize(image, (round(w*scale), round(h*scale)))
        self.face.setInputSize((small.shape[1], small.shape[0]))
        _, faces = self.face.detect(small)
        features = {}
        if faces is not None:
            for face in faces:
                face = face.copy()
                face[:14] /= scale
                x, y, fw, fh = face[:4]
                uv = ((x + fw/2)/w, (y + fh/2)/h)
                matches = [i for i, b in enumerate(boxes) if in_rect(uv, b)]
                if not matches:
                    boxes.append(np.clip([(x-fw*.6)/w, y/h, (x+fw*1.6)/w, (y+fh*3.5)/h], 0, 1).tolist())
                    matches = [len(boxes)-1]
                if len(matches) == 1 and fw >= 36:
                    crop = self.recognizer.alignCrop(image, face)
                    feature = self.recognizer.feature(crop).flatten()
                    feature /= max(np.linalg.norm(feature), 1e-8)
                    # Multiple faces in one body box cannot establish an identity.
                    i = matches[0]
                    features[i] = None if i in features else feature
        detected = []
        for i, (x1, y1, x2, y2) in enumerate(boxes):
            uv = [(x1+x2)/2, y1 + .30*(y2-y1)]
            depth, ratio, _ = measure(z, uv, ((x2-x1)*.12, (y2-y1)*.07))
            detected.append(dict(box=[x1,y1,x2,y2], uv=uv, feature=features.get(i), z=depth, valid=ratio))
        return detected


class Tracker:
    def __init__(self):
        self.tracks = {}
        self.next_id = 1
        self.selected = None
        self.selected_at = 0

    def update(self, detections, now, owner, profile):
        self.tracks = {i:t for i,t in self.tracks.items() if now-t["seen"] <= 1.5}
        matches = {}
        pairs = sorted(((overlap(d["box"], t["box"]), idx, i)
                        for idx,d in enumerate(detections) for i,t in self.tracks.items()), reverse=True)
        taken = set()
        for score, idx, i in pairs:
            if score >= .18 and idx not in matches and i not in taken:
                matches[idx] = i
                taken.add(i)
        visible = []
        for idx, d in enumerate(detections):
            i = matches.get(idx)
            if i is None:
                i = self.next_id
                self.next_id += 1
                self.tracks[i] = dict(id=i, role="待确认", votes=0, since=None, last_z=None,
                                      seen=now, visitor=False, ambiguous=False)
            t = self.tracks[i]
            ambiguous = any(j != idx and overlap(d["box"], item["box"]) > .35
                            for j, item in enumerate(detections))
            associations = [score for score, j, _ in pairs if j == idx and score >= .25]
            if len(associations) > 1 and associations[0] - associations[1] < .16:
                ambiguous = True
            if ambiguous or t["ambiguous"]:
                t.update(role="待确认", votes=0, visitor=False, since=None)
            if now-t["seen"] > .5:
                t["since"] = None
            t.update(d, seen=now, ambiguous=ambiguous)
            f = t["feature"]
            if owner and f is not None and not ambiguous:
                similarity = float(np.max(np.asarray(owner) @ f))
                if similarity >= .40:
                    t.update(role="主人", votes=0, visitor=False)
                elif similarity < .27 and t["role"] != "主人":
                    t["votes"] += 1
                    if t["votes"] >= 3:
                        t["role"] = "其他人"
                else:
                    t["votes"] = 0
            if t["z"] is not None:
                previous = t["last_z"]
                t["last_z"] = t["z"]
                if previous is not None and abs(t["z"]-previous) > max(.45, .3*previous):
                    t["z"] = None
            visible.append(t)
        owners = [t for t in visible if t["role"] == "主人"]
        visitors = []
        for t in visible:
            if t["role"] != "主人" and (owners or t["role"] == "其他人") and not t["ambiguous"]:
                t["visitor"] = True
            inside = in_rect(t["uv"], profile.get("roi", [0,0,1,1]))
            gate = profile.get("range")
            t["distance_pending"] = bool(gate and t["z"] is None)
            eligible = inside and (not gate or (t["z"] is not None and gate[0] <= t["z"] <= gate[1]))
            if t["visitor"] and eligible:
                if t["since"] is None:
                    t["since"] = now
                if now-t["since"] >= .5:
                    visitors.append(t)
            else:
                t["since"] = None
        if any(t["ambiguous"] for t in visible) or len(owners) > 1:
            return visible, None, "人物交叉，等待确认"
        previous = next((t for t in visible if t["id"] == self.selected), None)
        old_visitor = self.tracks.get(self.selected, {}).get("visitor", False)
        if not previous and now-self.selected_at <= 1.5 and self.selected is not None and (old_visitor or not visitors):
            return visible, None, "目标暂时丢失"
        current = next((t for t in visitors if t["id"] == self.selected), None)
        if not current and visitors:
            current = min(visitors, key=lambda t: (t["since"], t["id"]))
        if current:
            status = "有人进入"
        elif owners:
            current, status = owners[0], "主人"
        elif (len(visible) == 1 and not visible[0]["visitor"]
              and in_rect(visible[0]["uv"], profile.get("roi", [0,0,1,1]))
              and (not profile.get("range") or (visible[0]["z"] is not None
                   and profile["range"][0] <= visible[0]["z"] <= profile["range"][1]))):
            current, status = visible[0], "有人，身份待确认"
        else:
            status = "距离待确认" if any(t["distance_pending"] for t in visible) else "等待目标"
        if current:
            self.selected, self.selected_at = current["id"], now
        else:
            self.selected = None
        return visible, current, status
