"""Run from this folder with python -m unittest -v."""
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch
import threading

from robot import Robot

import cv2
import numpy as np

from geometry import aim, fit_samples, marker_pose, measure
from service import Service
from app import create_app
from vision import Tracker


def detection(x, feature=None, z=1.0):
    return dict(box=[x,.2,x+.15,.85], uv=[x+.075,.4], feature=feature, z=z, valid=1.)


def frame():
    return dict(bgr=np.zeros((480,640,3),np.uint8), z=np.ones((480,640),np.float32),
                K=np.float64([[500,0,320],[0,500,240],[0,0,1]]), dist=np.zeros(5),
                stamp=time.monotonic(), serial="test-camera", model="test", modes=[640,480,1,0,[640,480,2,2]],
                device_stamp=8, unit=.001)


class FollowTests(unittest.TestCase):
    def test_owner_visitor_hold(self):
        t = Tracker()
        owner = [[1.,0.]]
        def run(now, *items):
            return t.update(list(items), now, owner, {})[1:]
        self.assertEqual(run(10, detection(.1,np.array([1.,0.])))[1], "主人")
        self.assertEqual(run(10.2, detection(.1), detection(.7))[1], "主人")
        self.assertEqual(run(10.5, detection(.1), detection(.7))[1], "主人")
        current, status = run(10.8, detection(.1), detection(.7))
        self.assertEqual((current["id"],status), (2,"有人进入"))
        self.assertEqual(run(11, detection(.7))[1], "有人进入")
        self.assertIsNone(run(11.6,detection(.1))[0])
        self.assertEqual(run(12.6,detection(.1,np.array([1.,0.])))[1],"主人")
        self.assertIsNone(run(14.5)[0])

    def test_unseen_face_and_gates(self):
        t = Tracker()
        p = dict(roi=[.4,0,1,1], range=[.5,2])
        for i in range(8):
            _, target, status = t.update([detection(.1,np.array([1.,0.])),detection(.6,z=None)],
                                        10+i/5, [[1.,0.]], p)
            self.assertEqual(status,"主人")
        t = Tracker()
        for i in range(12):
            _, _, status = t.update([detection(.4)], i/5, [[1.,0.]], {})
        self.assertIn("身份待确认",status)

    def test_multiple_visitors_and_crossing(self):
        tracker = Tracker()
        owner = np.array([1.,0.])
        def update(stamp, positions):
            return tracker.update([detection(x, owner if x == .05 else None) for x in positions],
                                  stamp, [owner.tolist()], {})
        update(10,[.05,.40])
        update(10.3,[.05,.40,.70])
        _, target, _ = update(10.6,[.05,.40,.70])
        first = target["id"]
        _, target, _ = update(11,[.70,.40,.05])
        self.assertEqual(target["id"],first)
        _, target, status = update(11.2,[.05,.40,.43])
        self.assertIsNone(target)
        self.assertIn("交叉",status)

    def test_geometry_and_invalid_depth(self):
        f = frame()
        expected = [(x,y,60+40*x,75+22*y) for x,y in ((.5,.5),(.1,.5),(.9,.5),(.5,.1),(.5,.9))]
        matrix, rms = fit_samples(expected)
        self.assertLess(rms, 1e-8)
        with self.assertRaises(ValueError):
            fit_samples([expected[0]]*5)
        p = dict(regions={"seat":dict(rect=[.1,.1,.9,.9],matrix=matrix)})
        self.assertEqual(aim(f,p,[.25,.75],None),([70,92],"二维",False))
        self.assertIsNone(aim(f,p,[0,0],None)[0])
        p["pose"] = dict(axes=[[1,0,0],[0,0,-1],[0,1,0]],pivot=[-.2,0,0])
        near=aim(f,p,[.5,.5],1)
        far=aim(f,p,[.5,.5],3)
        self.assertEqual(near[1],"三维")
        self.assertGreater(near[0][0],far[0][0])
        self.assertEqual(aim(f,p,[.5,.5],None)[1],"二维")
        self.assertIsNone(measure(np.zeros((200,200)),[.5,.5])[0])

    def test_marker(self):
        f = frame()
        f["bgr"][:] = 255
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        square = cv2.aruco.generateImageMarker(dictionary, 9, 160)
        f["bgr"][160:320,240:400,:] = square[:,:,None]
        pose = marker_pose(f,dict(size=80,marker=9,right=0,forward=0,up=0,yaw=0))
        self.assertLess(pose["rms"],1.)
        self.assertAlmostEqual(pose["pivot"][2],.25,places=2)

    def test_command_rate_hold_and_lease_loss(self):
        calls = []
        robot = Robot()
        state = dict(robot="test", boot=42, generation=4)
        def route(op=None, **body):
            calls.append((op,body))
            return {"session":33} if op=="start" else state
        robot.state = state
        with patch.object(robot,"call",side_effect=route):
            robot.start()
            robot.send([92,88],10)
            robot.send([95,90],10.1)
            robot.send(None,10.21)
            robot.send(None,10.9)
            robot.send(None,11.3)
            robot.stop()
        updates=[body for op,body in calls if op=="update"]
        self.assertEqual([u["seq"] for u in updates],[1,2,3])
        self.assertEqual((updates[0]["x"],updates[0]["y"]),(92,88))
        self.assertTrue(updates[1]["hold"])
        self.assertEqual(updates[2],dict(session=33,seq=3))
        self.assertIsNone(robot.session)
        s=Service()
        s.robot.session=88
        with patch.object(s.robot,"call",return_value={}) as call:
            thread=threading.Thread(target=s.control_loop)
            thread.start()
            try:
                for _ in range(50):
                    if s.robot.session is None:
                        break
                    time.sleep(.01)
                call.assert_called_once_with("stop",session=88)
            finally:
                s.done.set()
                thread.join(1)

    def test_grid_targets_use_reported_limits_and_take_manual_control(self):
        service = Service()
        state = dict(minX=52, maxX=126, minY=72, maxY=108, targetX=94, targetY=87)
        service.robot.state = state
        service.robot.session = 33
        client = create_app(service).test_client()
        def submit(payload):
            return client.post("/api/manual", json=payload, headers={"X-Desk-Control":"1"})
        with patch.object(service.robot, "call", return_value=state) as call:
            for invalid in ({"x":60}, {"x":True,"y":80}, {"x":53.5,"y":80},
                            {"x":51,"y":80}, {"x":60,"y":109}, {"x":60,"y":80,"dx":1}):
                self.assertEqual(submit(invalid).status_code, 400)
                self.assertEqual(call.call_args.args, ())
                self.assertEqual(service.robot.session, 33)
            for x, y in ((52,72), (126,108), (90,90)):
                self.assertEqual(submit({"x":x,"y":y}).status_code, 200)
                call.assert_called_with("manual", x=x, y=y)
            self.assertIsNone(service.robot.session)
            self.assertEqual(submit({"dx":-1}).status_code, 200)
            call.assert_called_with("manual", x=93, y=87)
            self.assertEqual(submit({"center":True}).status_code, 200)
            call.assert_called_with("manual", x=90, y=90)

    def test_web_freeze_profile_and_expiry(self):
        with tempfile.TemporaryDirectory() as directory:
            s = Service()
            s.config = {}
            s.path = Path(directory)/"data.json"
            s.robot.state = dict(robot="test",x=90,y=90)
            f = frame()
            s.process_frame(f, [detection(.4,np.array([1.,0.]))])
            app=create_app(s).test_client()
            ident=s.counter
            self.assertEqual(app.get(f"/api/frame/{ident}?depth=1").mimetype,"image/jpeg")
            self.assertEqual(app.post("/api/start",json={}).status_code,403)
            def post(op, data):
                return app.post("/api/"+op,json=data,headers={"X-Desk-Control":"1"})
            self.assertEqual(post("freeze",{"frame":ident}).status_code,200)
            q=dict(frame=ident,uv=[.45,.4])
            self.assertEqual(post("measure",q).json["m"],1)
            self.assertTrue(post("enroll",q).json["ok"])
            self.assertEqual(len(s.config["owner"]),1)
            self.assertTrue(post("roi",dict(rect=[.1,0,.9,1])).json["ok"])
            f["stamp"] -= 10
            self.assertEqual(post("start",{}).status_code,400)
            self.assertIsNone(s.latest())
            f2=frame()
            f2["serial"]="different"
            s.process_frame(f2,[])
            self.assertFalse(s.profile())
            self.assertEqual(post("measure",q).status_code,400)


if __name__ == "__main__":
    unittest.main()
