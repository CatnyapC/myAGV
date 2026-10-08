"""Hardware-free acceptance checks for leases, STOP, task gates and ROS masks."""
import asyncio
from collections import OrderedDict
from contextlib import nullcontext
import json
import math
from pathlib import Path
import queue
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import uuid

from install_driver_watchdog import patch as patch_driver
from robot_control import Control, Stopped
from robot_ros import RobotROS
from robot_safety import footprint_clear, lease_valid, manual_vector, zones_visible
from web_backend.hardware import Hardware
from web_backend.photos import PhotoIndex, demo_frame


def fake_control():
    c = Control.__new__(Control)
    c.boot_id, c.stop_epoch = 'boot', 0
    c.phase, c.status, c.task_id = 'idle', 'Ready', None
    c.owner, c.deadline, c.input = None, 0, None
    c.executing = c.stopping = c.base_enabled = False
    c.stop_pending = None
    c.localized = True
    c.config = dict(transport_angles=[0, 10, 20], clearance_m=.25)
    c.arm, c.arm_homed, c.home_cancelled = Mock(), True, False
    c.capture = Mock()
    c.queue, c.results, c.requests = queue.Queue(maxsize=1), OrderedDict(), OrderedDict()
    c.robot = SimpleNamespace(grid=dict(map_id='map'), zones=dict(revision=2), zero=Mock(),
            nav=Mock(), costmap_received={}, layers_ok=True, driver_watchdog=True, exclusive=True, sensors_ready=lambda: True,
            zones_ready=lambda: True, clearance_ready=lambda r: True, validate_goal=Mock())
    return c


def packet(**values):
    return dict(op='command', boot_id='boot', stop_epoch=0, deadline=time.monotonic()+.45,
                id=str(uuid.uuid4()), type='home', session_id='tab', map_id='map', expected_revision=2, **values)


class ControlTest(unittest.TestCase):
    def test_manual_arm_step_sets_absolute_mode_before_target(self):
        c = fake_control()
        c.arm.get_coords_info.side_effect = [[200,0,0], [201,0,0]]
        c.arm.get_angles_info.return_value = [0,0,0]
        with patch('robot_control.arm_deadline', return_value=nullcontext()):
            c.arm_step('X', 1)
        calls = c.arm.method_calls
        self.assertLess(calls.index(unittest.mock.call.set_mode(0)), calls.index(unittest.mock.call.set_coords([201,0,0],30)))

    def test_retries_lease_and_stop_preempt_pending_command(self):
        c, p = fake_control(), packet()
        self.assertEqual(c.receive(p)['status'], 'accepted')
        self.assertEqual(c.receive(p)['status'], 'accepted')
        self.assertEqual(c.queue.qsize(), 1)
        with self.assertRaises(ValueError):
            c.receive(dict(p, type='gripper'))
        c.receive(dict(op='heartbeat', boot_id='boot', deadline=time.monotonic()+.45, session_id='other', input={'key':'forward'}))
        self.assertIsNone(c.input)
        c.receive(dict(op='stop'))
        self.assertEqual(c.deadline, 0)
        self.assertIsNotNone(c.stop_pending)
        c.robot.zero.assert_called()
        c.robot.nav.client.cancel_all_goals.assert_called()
        with self.assertRaises(ValueError):
            c.receive(dict(p, id=str(uuid.uuid4())))
        with self.assertRaises(ValueError):
            c.receive(dict(p, deadline=time.monotonic()-1))
        c.stop_hardware = Mock()
        self.assertFalse(c.base_allowed())

    def test_watchdog_and_unknown_goal_fail_closed(self):
        for fault in ('deadline', 'driver_watchdog', 'exclusive', 'sensors', 'zones'):
            c = fake_control()
            c.phase, c.base_enabled, c.deadline = 'navigating', True, time.monotonic()+.45
            if fault == 'deadline': c.deadline = 0
            elif fault in ('driver_watchdog', 'exclusive'): setattr(c.robot, fault, False)
            elif fault == 'sensors': c.robot.sensors_ready = lambda: False
            else: c.robot.zones_ready = lambda: False
            c.watchdog()
            self.assertFalse(c.base_allowed(), fault)
        c = fake_control()
        with self.assertRaises(Stopped): c.travel_guard(dict(x_m=0,y_m=0))
        c.deadline = time.monotonic()+.45
        c.localized = False
        with self.assertRaises(RuntimeError): c.travel_guard(dict(x_m=0,y_m=0))

    def test_fetch_checks_before_movement_and_requires_both_reviews(self):
        from web_backend.photos import digest_station
        c = fake_control()
        c.stations_path = Path('unused')
        c.travel_guard = Mock()
        c.fold = Mock()
        c.grip = Mock()
        c.move_arm = Mock()
        c.review = Mock()
        c.go = Mock()
        record = dict(base=dict(x_m=0,y_m=0,yaw_deg=0), arm_angles_deg=[0,20,30])
        data = dict(station='cup', station_digest=digest_station(record))
        with patch('robot_control.load_stations', return_value={'cup':record}), patch('robot_control.wait_arm', return_value=[0,10,20,0]):
            with self.assertRaises(ValueError): c.fetch(data)
            c.fold.assert_not_called()
        with patch('robot_control.load_stations', return_value={'cup':record}), patch('robot_control.wait_arm', return_value=[0,10,20]):
            c.fetch(data)
        self.assertEqual([x.args[0] for x in c.review.call_args_list], ['review_grasp','verify_grasp'])
        self.assertEqual([x.args[0] for x in c.grip.call_args_list], [100,0,100])
        c.fold.reset_mock()
        c.robot.nav.go_to_pickup.reset_mock()
        c.capture.side_effect = RuntimeError('Arm camera unavailable')
        with patch('robot_control.load_stations', return_value={'cup':record}), patch('robot_control.wait_arm', return_value=[0,10,20]):
            with self.assertRaises(RuntimeError): c.fetch(data)
        c.fold.assert_not_called()
        c.robot.nav.go_to_pickup.assert_not_called()

    def test_review_rejects_wrong_stage_or_task(self):
        c = fake_control()
        c.phase, c.task_id, c.owner = 'verify_grasp', 'task', 'tab'
        with self.assertRaises(RuntimeError): c.receive(dict(packet(),type='confirm',task_id='wrong',stage='verify_grasp'))
        with self.assertRaises(RuntimeError): c.receive(dict(packet(),type='align_step'))
        c.receive(dict(packet(),type='confirm',task_id='task',stage='verify_grasp'))
        self.assertEqual(c.confirmation, 'verify_grasp')

    def test_unconfirmed_stop_latches_motion_fault(self):
        c = fake_control()
        c.phase = 'homing'
        with patch('robot_control.arm_deadline', return_value=nullcontext()), patch('robot_control.wait_arm', return_value=[0,0,0]):
            c.stop_hardware('Stopped')
            self.assertEqual(c.phase, 'fault')
            self.assertFalse(c.arm_homed)
            c.stop_hardware('Stopped again')
            self.assertEqual(c.phase, 'fault')
        with self.assertRaises(RuntimeError): c.receive(packet())
        c.receive(dict(packet(), type='recover_stop', confirmed=True))
        self.assertEqual(c.queue.qsize(), 1)

    def test_pure_geometry_and_driver_patch(self):
        grid = dict(width=10,height=10,resolution_m=.1, origin=dict(x_m=1,y_m=2,yaw_rad=math.pi/2),cells=[0]*100)
        pose=dict(x_m=.5,y_m=2.5)
        self.assertTrue(footprint_clear(grid,pose,.15))
        grid['cells'][55]=-1
        self.assertFalse(footprint_clear(grid,pose,.15))
        self.assertFalse(zones_visible(grid, [[.45,2.55]]))
        grid['cells'][55]=100
        self.assertTrue(zones_visible(grid, [[.45,2.55]]))
        self.assertFalse(lease_valid(float('inf'),0))
        self.assertFalse(lease_valid(.61,0))
        self.assertEqual(manual_vector('PICKUP','forward'),(.03,0,0))
        with self.assertRaises(ValueError): manual_vector('PICKUP','left')
        src='double angularZ = 0.0;\nlinearX = msg.linear.x;\nros::Rate loop_rate(100);\nmyAGV.execute(linearX, linearY, angularZ);\n\treturn 0;'
        result=patch_driver(src)
        self.assertIn('> 0.5',result)
        self.assertIn('ros::WallRate loop_rate',result)
        self.assertEqual(patch_driver(result.replace('ros::WallRate', 'ros::Rate')), result)
        self.assertEqual(patch_driver(result),result)
        with self.assertRaises(ValueError): patch_driver('unknown driver')

    def test_real_photo_metadata_roundtrip(self):
        with tempfile.TemporaryDirectory() as tmp:
            photos=PhotoIndex(Path(tmp),Path(tmp)/'stations.json')
            meta=dict(kind='observation',source='ros',camera_id='front',base_pose=dict(x_m=1,y_m=2,yaw_rad=.5),
                      map_id='ros-map',frame='map',captured_at_s=1234,map_revision=1,capture_id=str(uuid.uuid4()),timestamp_basis='host_frame_receipt')
            photos.add(demo_frame('front'),meta,0)
            restored=PhotoIndex(Path(tmp),Path(tmp)/'stations.json').snapshot('ros-map')['photos'][0]
            for k,v in meta.items(): self.assertEqual(restored[k],v)


class IPCtest(unittest.IsolatedAsyncioTestCase):
    async def test_challenge_replay_disconnect_and_no_socket_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            h=Hardware(tmp)
            h.control=dict(boot_id='boot',stamp_s=time.time(),phase='idle')
            calls=[]
            async def rpc(p): calls.append(p); return dict(status='accepted')
            h.rpc=rpc
            nonce=h.challenge('tab')['nonce']
            h.owner='tab'
            self.assertTrue(await h.heartbeat('tab',dict(nonce=nonce)))
            self.assertFalse(await h.heartbeat('tab',dict(nonce=nonce)))
            self.assertEqual(len(calls),1)
            await h.command(dict(packet(),session_id='tab'))
            self.assertEqual(calls[-1]['op'],'command')
            await h.disconnect('tab')
            self.assertEqual(calls[-1]['op'],'stop')
            with self.assertRaises(ValueError): await h.command(dict(packet(),session_id='tab'))
            with self.assertRaises(RuntimeError): await Hardware(tmp).stop()

    async def test_real_unix_protocol_can_stop_while_worker_is_busy(self):
        with tempfile.TemporaryDirectory() as tmp:
            c=fake_control()
            c.directory=Path(tmp)
            import threading
            c.receive_lock=threading.Lock()
            c.start_socket()
            h=Hardware(tmp)
            try:
                self.assertEqual((await h.rpc(packet()))['status'],'accepted')
                self.assertEqual((await h.stop())['status'],'stopping')
                self.assertIsNotNone(c.stop_pending)
                self.assertEqual((await h.rpc(dict(op='result',id='missing')))['status'],'pending')
            finally:
                await asyncio.to_thread(c.server.shutdown)
                c.server.server_close()


if __name__ == '__main__':
    unittest.main()
