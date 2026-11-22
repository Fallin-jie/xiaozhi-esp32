"""Run from this folder with python -m unittest -v."""
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch
import threading

from robot import Robot

import cv2
import numpy as np

from camera import color_geometry, select_camera
import pyrealsense2 as rs

from geometry import aim, fit_samples, marker_pose, measure
from service import Service
from speech import DEFAULT_ALERT
from app import create_app
from vision import Detector, Tracker


def detection(x, feature=None, z=1.0):
    return dict(box=[x,.2,x+.15,.85], uv=[x+.075,.4], feature=feature, z=z, valid=1.)


def frame():
    return dict(bgr=np.zeros((480,640,3),np.uint8), z=np.ones((480,640),np.float32),
                K=np.float64([[500,0,320],[0,500,240],[0,0,1]]), dist=np.zeros(5),
                stamp=time.monotonic(), serial="test-camera", model="test", modes=[640,480,1,0,[640,480,2,2]],
                device_stamp=8, unit=.001)


class FollowTests(unittest.TestCase):
    def test_d405_selection(self):
        devices = [dict(model="RealSense D405", serial=sn) for sn in ("A", "B")]
        self.assertEqual(select_camera(devices[:1], ""), devices[0])
        self.assertEqual(select_camera(devices, "B"), devices[1])
        for choices, serial in (([], ""), (devices, ""), (devices, "missing")):
            with self.assertRaises(ValueError):
                select_camera(choices, serial)

    def test_d405_rectified_pixels_follow_sdk_rays(self):
        intr = rs.intrinsics()
        intr.width, intr.height = 32, 24
        intr.fx, intr.fy, intr.ppx, intr.ppy = 22., 23., 15., 12.
        intr.model = rs.distortion.inverse_brown_conrady
        intr.coeffs = [-.05, .06, .0003, .0015, -.022]
        K, pixels = color_geometry(intr)
        for u, v in ((1, 1), (15, 12), (30, 22)):
            ray = rs.rs2_deproject_pixel_to_point(intr, pixels[v, u].tolist(), 1.)
            np.testing.assert_allclose(K @ ray, [u, v, 1.], atol=.001)

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

    def test_no_owner_tracks_first_stable_person_with_no_face(self):
        tracker = Tracker()
        gate = dict(roi=[.3,0,1,1], range=[.5,2])
        def see(t, *people):
            return tracker.update(list(people), t, [], gate)[1:]
        self.assertIsNone(see(1, detection(.1), detection(.4,z=None))[0])
        self.assertIsNone(see(1.3, detection(.1), detection(.4))[0])
        self.assertIsNone(see(1.6, detection(.4), detection(.7))[0])
        first, status = see(1.9, detection(.4), detection(.7))
        self.assertEqual(status, "有人进入")
        self.assertEqual(first["box"][0], .4)
        self.assertEqual(see(2.2, detection(.7), detection(.42))[0]["id"], first["id"])
        self.assertIsNone(see(2.4, detection(.7))[0])
        for t in (2.7,3.1,3.5,3.8):
            target, status = see(t, detection(.7))
        self.assertEqual((target["box"][0],status), (.7,"有人进入"))
        self.assertIsNone(see(4.0, detection(.7,z=3))[0])

    def test_visitor_voice_announces_arrivals_once_without_follow(self):
        s = Service()
        s.config = {}
        s.robot.state = {"robot": "test", "boot": 1, "visitorAlert": 0}
        f = frame()
        def observe(t, *detections):
            snap = dict(f, stamp=f["stamp"] + t)
            s.process_frame(snap, list(detections))
            s.announce_visitors(snap["stamp"])

        with patch.object(s.robot, "announce_visitor", side_effect=[False, True, True, True]) as say:
            person, newcomer = detection(.2), detection(.7)
            for i in range(3):
                observe(i*.2, person)
            say.assert_not_called()
            for i in range(3, 16):
                observe(i*.2, person)
            self.assertEqual(say.call_count, 2)
            self.assertEqual(say.call_args_list[0], say.call_args_list[1])
            for i in range(16, 24):
                observe(i*.2, person, newcomer)
            self.assertEqual(say.call_count, 3)
            observe(4.8)
            for i in range(25, 32):
                observe(i*.2, person, newcomer)
            self.assertEqual(say.call_count, 3)
            observe(10)
            for i in range(51, 61):
                observe(i*.2, person, newcomer)
            self.assertEqual(say.call_count, 4)
            self.assertIsNone(s.robot.session)

    def test_visitor_voice_obeys_identity_region_distance_and_frame_freshness(self):
        s = Service()
        s.config = {"owner": [[1., 0.]]}
        s.robot.state = {"robot": "test", "boot": 1, "visitorAlert": 0}
        f = frame()
        s.process_frame(f, [])
        s.profile().update(roi=[.4,0,1,1], range=[.5,2])
        with patch.object(s.robot, "announce_visitor", return_value=True) as say:
            for i in range(12):
                snap = dict(f, stamp=f["stamp"] + i*.2)
                people = [detection(.5, np.array([1.,0.])), detection(.1), detection(.8,z=3)]
                s.process_frame(snap, people)
                s.announce_visitors(snap["stamp"])
            say.assert_not_called()
            for i in range(6):
                s.process_frame(dict(f, stamp=f["stamp"]-10+i*.2), [detection(.6, np.array([0.,1.]))])
            self.assertTrue(s.tracker.visitors)
            s.announce_visitors(time.monotonic())
            say.assert_not_called()

    def test_saved_alert_survives_restart_and_manual_text_plays_separately(self):
        with tempfile.TemporaryDirectory() as folder, patch("service.DATA", Path(folder)):
            s = Service()
            s.robot.address = "127.0.0.1"
            s.robot.state = {"robot": "test", "boot": 3, "textVoice": True, "visitorAlert": 0}
            client = create_app(s).test_client()
            def submit(op, value):
                return client.post("/api/" + op, json={"text": value}, headers={"X-Desk-Control": "1"})
            with patch("service.synthesize", side_effect=lambda t: t.encode()), patch("service.play_voice") as play:
                for invalid in ("   ", "长" * 201, None):
                    self.assertEqual(submit("alert_text", invalid).status_code, 400)
                self.assertEqual(submit("alert_text", "门口有访客").status_code, 200)
                play.assert_not_called()
                restored = Service()
                self.assertEqual(restored.state()["alert_text"], "门口有访客")
                self.assertEqual(restored.visitor_clip, "门口有访客".encode())
                self.assertEqual(submit("speak", "你好").status_code, 200)
                self.assertEqual(play.call_args.args[3:], ("manual", "你好".encode()))
                f = frame()
                for i in range(8):
                    now = f["stamp"] + i * .2
                    s.process_frame(dict(f, stamp=now), [detection(.4)])
                    s.announce_visitors(now)
                self.assertEqual(play.call_count, 2)
                self.assertEqual(play.call_args.args[3:], ("visitor", restored.visitor_clip))
                with patch.object(s, "save", side_effect=OSError("disk")):
                    self.assertEqual(submit("alert_text", "新词").status_code, 400)
                self.assertEqual(s.state()["alert_text"], "门口有访客")
                self.assertEqual(submit("alert_text", DEFAULT_ALERT).status_code, 200)
                self.assertEqual(Service().visitor_clip, b"")

    def test_voice_transport_failure_preserves_follow_session(self):
        r = Robot()
        r.state = {"boot": 1, "robot": "test"}
        r.session = 77
        with patch.object(r, "_request", side_effect=TimeoutError):
            self.assertFalse(r.announce_visitor(12))
        self.assertEqual(r.session, 77)

    def test_no_owner_generates_angles_without_calibration(self):
        s = Service()
        s.config = {}
        s.robot.state = {"robot":"test"}
        f = frame()
        stamp = f["stamp"]
        for idx, x in enumerate((.1,.1,.1,.12,.16,.2)):
            f = dict(f, stamp=stamp+idx*.25)
            s.process_frame(f, [detection(x, z=None)])
            if idx == 2:
                initial_x = s.history[-1]["angles"][0]
        snap = s.history[-1]
        self.assertEqual(snap["status"], "有人进入")
        self.assertEqual(snap["method"], "画面方向（估算）")
        self.assertGreater(initial_x, snap["angles"][0])
        self.assertGreater(snap["angles"][0], 90)
        # Replay a target at symmetric offsets from the optical center, at the same height.
        for pixel, expected in ((170,[107,86]), (320,[90,85]), (470,[73,86])):
            self.assertEqual(aim(f, {}, [pixel/639,200/479], None, camera_direction=True)[0], expected)
        self.assertEqual(aim(f, {}, [1,1], None, camera_direction=True)[0][1], 110)
        self.assertIsNone(aim(f, {}, [.2,.4], None)[0])
        partial = {"regions":{"seat":{"rect":[0,0,.5,.5],"samples":[]}}}
        self.assertIsNone(aim(f, partial, [.2,.4], None, camera_direction=True)[0])
        s.process_frame(dict(f, stamp=f["stamp"]+.2), [])
        self.assertIsNone(s.history[-1]["angles"])

    def test_stranger_replaces_owner_in_same_box(self):
        tracker = Tracker()
        owner = np.array([1., 0.])
        other = np.array([0., 1.])
        def observe(t, feature):
            return tracker.update([detection(.3, feature)], t, [owner.tolist()], {})
        self.assertEqual(observe(1, owner)[2], "主人")
        self.assertEqual(observe(1.1, None)[2], "主人")
        self.assertEqual(observe(1.2, other)[0][0]["role"], "待确认")
        observe(1.3, None)
        observe(1.4, other)
        self.assertEqual(observe(1.5, other)[0][0]["role"], "待确认")
        self.assertEqual(observe(1.6, other)[0][0]["role"], "其他人")
        observe(1.9, other)
        self.assertEqual(observe(2.2, other)[2], "有人进入")
        self.assertEqual(observe(2.3, owner)[2], "主人")
        self.assertEqual(observe(3.1, None)[0][0]["role"], "待确认")

    def test_two_faces_in_one_body_cannot_enroll_or_alert(self):
        detector = Detector.__new__(Detector)
        detector.bodies = Mock(return_value=[[.1, .1, .8, .95]])
        detector.face, detector.recognizer = Mock(), Mock()
        # The smaller face still makes this body ambiguous.
        faces = np.zeros((2, 15), dtype=np.float32)
        faces[:, :4] = [[100, 70, 80, 82], [230, 90, 28, 30]]
        detector.face.detect.return_value = None, faces
        items = detector.detect(frame()["bgr"], None)
        detector.recognizer.feature.assert_not_called()
        tracks, target, _ = Tracker().update(items, 1, [[1., 0.]], {})
        self.assertTrue(tracks[0]["ambiguous"])
        self.assertIsNone(target)

    def test_enroll_only_the_person_in_the_displayed_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            s = Service()
            s.config = {}
            s.path = Path(directory)/"settings.json"
            s.process_frame(frame(), [detection(.1, np.array([1.,0.])), detection(.6, np.array([0.,1.]))])
            ident = s.counter
            with self.assertRaisesRegex(ValueError, "多人"):
                s.command("enroll", {"frame":ident})
            s.command("enroll", {"frame":ident, "uv":[.15,.4]})
            s.process_frame(frame(), [detection(.6, np.array([0.,1.]))])
            with self.assertRaisesRegex(ValueError, "不匹配"):
                s.command("enroll", {"frame":s.counter})
            s.process_frame(frame(), [detection(.1, np.array([.96,.28]))])
            s.command("enroll", {"frame":s.counter})
            self.assertEqual(s.state()["owner"], 2)

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
        # Avoid a half-degree tie after the floating-point least-squares fit.
        self.assertEqual(aim(f,p,[.25,.76],None),([70,92],"二维",False))
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

    def test_calibrated_center_respects_existing_2d_samples_and_limits(self):
        p = dict(regions={"seat":dict(rect=[0,0,1,1], matrix=[[0,0],[0,0],[125,90]])})
        robot = dict(centerOffsetX=8,centerOffsetY=-4,minX=50,maxX=122,minY=74,maxY=110)
        self.assertEqual(aim(frame(), p, [.5,.5], None, robot=robot), ([117,94],"二维",False))
        p["regions"]["seat"]["matrix"][2][0] = 135
        self.assertEqual(aim(frame(), p, [.5,.5], None, robot=robot), ([122,94],"二维",True))

    def test_center_calibration_uses_displayed_generation_and_requires_manual_rest(self):
        s = Service()
        st = dict(x=93,y=89,boot=42,generation=7,mode="manual",moving=False,centerOffsetX=0,centerOffsetY=0)
        s.robot.state = st
        seen = dict(boot=42,generation=7)
        with patch.object(s.robot,"call") as call:
            s.command("save_center", seen)
            call.assert_called_once_with("save_center", **seen)
            for change in (dict(mode="auto"),dict(moving=True),dict(generation=8),dict(x=108)):
                s.robot.state = dict(st, **change)
                with self.assertRaises(ValueError):
                    s.command("save_center", seen)
            call.assert_called_once()

    def test_manual_speed_accepts_integer_range_and_legacy_presets(self):
        service = Service()
        client = create_app(service).test_client()
        def submit(value):
            return client.post("/api/speed", json={"value": value}, headers={"X-Desk-Control": "1"})

        service.robot.state = dict(manualSpeed=10, manualSpeedMin=1, manualSpeedMax=120)
        with patch.object(service.robot, "call") as call:
            for speed in (1, 13, 47, 120):
                call.return_value = dict(service.robot.state, manualSpeed=speed)
                self.assertEqual(submit(speed).get_json(), {"robot": call.return_value})
                call.assert_called_with("speed", value=speed)
            call.reset_mock()
            for invalid in (0, 121, 3.5, True, "17", None):
                self.assertEqual(submit(invalid).status_code, 400)
            service.robot.state = dict(manualSpeed=15)
            self.assertEqual(submit(12).status_code, 400)
            call.assert_not_called()
            for speed in (5, 10, 15, 20):
                call.return_value = dict(manualSpeed=speed)
                self.assertEqual(submit(speed).status_code, 200)
                call.assert_called_with("speed", value=speed)
            service.robot.state = {}
            call.reset_mock()
            self.assertEqual(submit(10).status_code, 400)
            call.assert_not_called()

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

    def test_lan_access_is_opt_in_and_control_guard_is_preserved(self):
        lan_ip = "10.100.10.4"
        url = f"http://{lan_ip}:8765"
        peer = {"REMOTE_ADDR": "10.100.2.20"}
        self.assertEqual(create_app(None).test_client().get("/", base_url=url, environ_overrides=peer).status_code, 403)
        client = create_app(None, lan_ip).test_client()
        with client.get("/", base_url=url, environ_overrides=peer) as response:
            self.assertEqual(response.status_code, 200)
        self.assertEqual(client.get("/", base_url="http://localhost", environ_overrides=peer).status_code, 403)
        self.assertEqual(client.get("/", base_url="http://unknown.test", environ_overrides=peer).status_code, 403)
        self.assertEqual(client.get("/", base_url=url, environ_overrides={"REMOTE_ADDR": "203.0.113.1"}).status_code, 403)
        self.assertEqual(client.post("/api/start", base_url=url, environ_overrides=peer, json={}).status_code, 403)

    def test_mirrored_preview_preserves_depth_and_box_alignment(self):
        s = Service()
        s.config = {}
        f = frame()
        f["bgr"][:, :160] = [10,60,220]
        f["z"][:] = 0
        f["z"][:, :160] = .3
        s.process_frame(f, [detection(.1)])
        client = create_app(s).test_client()
        for depth in (0,1):
            def preview(mirror):
                path = f"/api/frame/{s.counter}?depth={depth}&mirror={mirror}"
                jpg = client.get(path).data
                return cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            native, mirrored = preview(0), preview(1)
            np.testing.assert_allclose(mirrored[300,550], native[300,89], atol=5)
            self.assertGreater(int(mirrored[300,550].max()), 80)
            self.assertLess(int(mirrored[300,89].max()), 8)
            # The body's left edge in the sensor frame is its right edge in the preview.
            np.testing.assert_allclose(mirrored[280,575], native[280,64], atol=22)
            self.assertGreater(int(mirrored[280,575,1]), 145)

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
