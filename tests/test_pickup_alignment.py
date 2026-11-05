import math
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from pickup_alignment import PickupAlignment, corridor_clear, staging_pose


def pose(x=0, y=0, yaw=0):
    return dict(x_m=x, y_m=y, yaw_deg=yaw)


def twist():
    return NS(linear=NS(x=0, y=0, z=0), angular=NS(x=0, y=0, z=0))


def grid():
    return NS(info=NS(width=100, height=100, resolution=0.05,
                     origin=NS(position=NS(x=-2.5, y=-2.5),
                               orientation=NS(x=0, y=0, z=0, w=1))), data=[0] * 10000)


class PickupAlignmentTest(unittest.TestCase):
    def setUp(self):
        self.d = PickupAlignment.__new__(PickupAlignment)
        self.d.distance, self.d.speed, self.d.clearance = 0.3, 0.03, 0.25
        self.d.nav = Mock()
        self.d.nav.ros.is_shutdown.return_value = False
        self.d.nav.twist_type = twist
        self.d.exclusive = Mock()
        self.d.check_clear = Mock()

    def test_staging_uses_recorded_heading(self):
        for angle in (0, 90, -90, 179):
            p = pose(1, 2, angle)
            s = staging_pose(p, 0.3)
            self.assertAlmostEqual(math.hypot(s['x_m'] - 1, s['y_m'] - 2), 0.3)
            self.assertAlmostEqual(s['x_m'] + 0.3 * math.cos(math.radians(angle)), 1)
            self.assertAlmostEqual(s['y_m'] + 0.3 * math.sin(math.radians(angle)), 2)
            self.assertEqual(s['yaw_deg'], angle)

    def test_forward_reverse_never_rotate_and_always_stop(self):
        for direction, positions, target in ((1, [0, 0.15, 0.295, 0.30], pose(0.3)),
                                             (-1, [0.3, 0.15, 0.005, 0], pose())):
            with self.subTest(direction=direction), patch('pickup_alignment.time.sleep'):
                nav = self.d.nav
                nav.publisher.reset_mock()
                nav.get_pose.side_effect = [pose(x) for x in positions]
                self.d.align(target)
                commands = [c.args[0] for c in nav.publisher.publish.call_args_list]
                self.assertTrue(any(c.linear.x == direction * 0.03 for c in commands))
                self.assertTrue(all(c.angular.z == c.linear.y == 0 for c in commands))
                self.assertEqual([c.linear.x for c in commands[-3:]], [0, 0, 0])
                nav.wait_stopped.assert_called()

    def test_misalignment_localization_jump_and_missing_pose_stop(self):
        for actual in (pose(0, 0.03), pose(0, 0, 16), pose(0.7), pose(-1), RuntimeError('lost TF')):
            with self.subTest(actual=actual), patch('pickup_alignment.time.sleep'):
                nav = self.d.nav
                nav.publisher.reset_mock()
                nav.get_pose.side_effect = [actual]
                with self.assertRaises(RuntimeError):
                    self.d.align(pose(0.3))
                self.assertTrue(all(c.args[0].linear.x == 0 for c in nav.publisher.publish.call_args_list))

    def test_obstacle_mid_approach_stops_without_retry(self):
        nav = self.d.nav
        nav.get_pose.side_effect = [pose(), pose(0.01)]
        self.d.check_clear.side_effect = [None, RuntimeError('blocked')]
        with patch('pickup_alignment.time.sleep'), self.assertRaisesRegex(RuntimeError, 'blocked'):
            self.d.align(pose(0.3))
        commands = [c.args[0] for c in nav.publisher.publish.call_args_list]
        self.assertEqual([c.linear.x for c in commands], [0.03, 0, 0, 0])

    def test_no_progress_is_bounded(self):
        now = [0.0]
        self.d.nav.get_pose.return_value = pose()
        def sleep(seconds):
            now[0] += seconds
        with patch('pickup_alignment.time.monotonic', side_effect=lambda: now[0]), patch('pickup_alignment.time.sleep', side_effect=sleep):
            with self.assertRaisesRegex(RuntimeError, 'no measured progress'):
                self.d.align(pose(0.3))
        self.assertLess(now[0], 4)
        self.assertEqual(self.d.nav.publisher.publish.call_args.args[0].linear.x, 0)

    def test_approach_can_stage_ahead_or_behind(self):
        self.d.align = Mock()
        for start, stage in ((0, 0.7), (2, 1.3)):
            self.d.nav.get_pose.return_value = pose(start)
            self.d.approach(pose(1), 30)
            self.d.nav.go_to.assert_called_with(pose(stage), 30)
            self.d.align.assert_called_with(pose(1))

    def test_heading_correction_and_overshoot_reverse(self):
        nav = self.d.nav
        nav.get_pose.side_effect = [pose(0, 0, -4), pose(), pose(0.32), pose(0.3), pose(0.3)]
        with patch('pickup_alignment.time.sleep'):
            self.d.align(pose(0.3))
        cmds = [c.args[0] for c in nav.publisher.publish.call_args_list]
        self.assertEqual(cmds[0].linear.x, 0)
        self.assertEqual(cmds[0].angular.z, 0.05)
        self.assertEqual(cmds[1].linear.x, 0.03)
        self.assertEqual(cmds[2].linear.x, -0.03)
        self.assertTrue(all(abs(c.angular.z) <= 0.05 for c in cmds))
        self.assertEqual([c.linear.x for c in cmds[-3:]], [0, 0, 0])

    def test_rotation_is_collision_checked_and_final_pose_checked(self):
        nav = self.d.nav
        nav.get_pose.side_effect = [pose(0, 0, 5)]
        self.d.check_clear.side_effect = RuntimeError('blocked rotation')
        with patch('pickup_alignment.time.sleep'), self.assertRaisesRegex(RuntimeError, 'blocked'):
            self.d.align(pose(0.3))
        self.d.check_clear.assert_called_once_with(0)
        self.assertTrue(all(c.args[0].angular.z == 0 for c in nav.publisher.publish.call_args_list))
        nav.get_pose.side_effect = [pose(0.3), pose(0.35)]
        with patch('pickup_alignment.time.sleep'), self.assertRaisesRegex(RuntimeError, 'final pose'):
            self.d.align(pose(0.3))

    def test_costmap_rejects_obstacle_unknown_edges_both_directions(self):
        g = grid()
        self.assertTrue(corridor_clear(g, 0, 0, 0, 1, 0.25))
        for direction in (1, -1):
            col = math.floor((direction * 0.3 + 2.5) / 0.05)
            for cost in (100, -1, 20):
                g.data[50 * 100 + col] = cost
                self.assertFalse(corridor_clear(g, 0, 0, 0, direction, 0.25))
            g.data[50 * 100 + col] = 0
        self.assertFalse(corridor_clear(g, 2.4, 0, 0, 1, 0.25))

    def test_competing_controller_and_active_goal_refused(self):
        nav = self.d.nav
        nav.ros.get_name.return_value = '/myagv_fetch'
        nav.ros.resolve_name.return_value = '/cmd_vel'
        master = Mock()
        master.getSystemState.return_value = [[['/cmd_vel', ['/move_base', '/teleop']]], [], []]
        with patch.dict('sys.modules', rosgraph=NS(Master=Mock(return_value=master))):
            with self.assertRaisesRegex(RuntimeError, 'Close other'):
                PickupAlignment.exclusive(self.d)
            master.getSystemState.return_value = [[['/cmd_vel', ['/move_base', '/myagv_fetch']]], [], []]
            nav.client.get_state.return_value = 1
            with self.assertRaisesRegex(RuntimeError, 'active'):
                PickupAlignment.exclusive(self.d)
            nav.client.get_state.return_value = 3
            PickupAlignment.exclusive(self.d)

    def test_missing_stale_and_unusable_feedback_refused(self):
        class Stamp:
            def __init__(self, seconds=0):
                self.seconds = seconds

            @staticmethod
            def now():
                return Stamp(10)

            def __sub__(self, other):
                return NS(to_sec=lambda: self.seconds - other.seconds)

        d, nav = self.d, self.d.nav
        nav.ros.Time = Stamp
        g = grid()
        g.header = NS(frame_id='odom', stamp=Stamp(10))
        scan = NS(header=NS(stamp=Stamp(10)), range_min=0.1, range_max=10, ranges=[2])
        odom = NS(header=NS(stamp=Stamp(10)))
        transform = NS(header=NS(stamp=Stamp(10)), transform=NS(
            translation=NS(x=0, y=0), rotation=NS(x=0, y=0, z=0, w=1)))
        nav.tf.lookup_transform.return_value = transform
        with patch('pickup_alignment.time.monotonic', return_value=10):
            d.grid, d.scan, nav.odom = (10, g), (10, scan), (10, odom)
            PickupAlignment.check_clear(d, 1)
            for sample in (g, scan, odom, transform):
                sample.header.stamp = Stamp(8)
                with self.assertRaisesRegex(RuntimeError, 'stale'):
                    PickupAlignment.check_clear(d, 1)
                sample.header.stamp = Stamp(10)
            d.grid = (8, g)
            with self.assertRaisesRegex(RuntimeError, 'stale'):
                PickupAlignment.check_clear(d, 1)
            d.grid = None
            with self.assertRaisesRegex(RuntimeError, 'fresh'):
                PickupAlignment.check_clear(d, 1)
            d.grid = (10, g)
            scan.ranges = [float('inf'), float('nan')]
            with self.assertRaisesRegex(RuntimeError, 'no usable'):
                PickupAlignment.check_clear(d, 1)


if __name__ == '__main__':
    unittest.main()
