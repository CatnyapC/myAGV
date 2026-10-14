"""Measured millimeter calibration and independent controller stop criteria."""
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from fetch_calibration import MetricArmCalibration, metric_arm_alignment
from test_robot_control import fake_control


class MetricAlignmentTest(unittest.TestCase):
    def test_full_residual_gate_precedes_gain_and_limits(self):
        for x, y, aligned in ((.7,.5,True), (1.24,0,True), (1.25,0,False), (1,-1,False), (20,0,False)):
            with self.subTest(x=x, y=y):
                moves, done = metric_arm_alignment(dict(x_mm=x, y_mm=y, aligned=math.hypot(x,y)<1), dict(max_step_mm=.5))
                self.assertEqual(done, aligned)
                self.assertEqual(not any(moves.values()), aligned)
                self.assertLessEqual(abs(moves['x_mm']), .5)
        self.assertEqual(metric_arm_alignment(dict(x_mm=50, y_mm=-30, aligned=False), None)[0], dict(x_mm=50,y_mm=-30,turn_deg=0))
        self.assertEqual(metric_arm_alignment(dict(x_mm=36.015625,y_mm=-7.395833333,aligned=False), dict(max_step_mm=0))[0], dict(x_mm=36.015625,y_mm=-7.395833333,turn_deg=0))
        for bad in (None, True, float('nan'), float('inf')):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                metric_arm_alignment(dict(x_mm=bad,y_mm=0,aligned=False), None)
        with self.assertRaises(ValueError):
            metric_arm_alignment(dict(x_mm=1,y_mm=0,aligned=True), None)

    def test_independent_measured_probes_freeze_goal_and_learn_cross_coupling(self):
        state = MetricArmCalibration()
        goal = [350,750,650,1000]
        moves, done, residual = state.observe(dict(goal=goal,current=[450,700,750,1000]), [0,150,-43.69], None)
        self.assertEqual(moves, dict(x_mm=5,y_mm=0,turn_deg=0))
        self.assertFalse(done)
        self.assertIsNone(residual)
        moves, _, _ = state.observe(dict(goal=[300,770,600,1000], current=[410,705,710,1000]), [0,155,-43.69], None)
        self.assertEqual(moves, dict(x_mm=0,y_mm=-5,turn_deg=0))
        moves, done, residual = state.observe(dict(goal=goal,current=[400,755,700,1000]), [5,155,-43.69], None)
        self.assertEqual(state.goal, goal)
        self.assertEqual(state.context()['phase'], 'metric')
        self.assertAlmostEqual(residual['x_mm'], 510/78)
        self.assertAlmostEqual(residual['y_mm'], 90/78)
        self.assertFalse(done)
        self.assertAlmostEqual(moves['x_mm'], 510/78)

    def test_successful_goal_scale_can_be_restored_without_probes(self):
        saved = dict(goal_bounds=[350,750,650,1000], image_error_to_remaining_mm=[[.125,0],[0,.1]])
        restored = MetricArmCalibration(saved)
        self.assertTrue(restored.reused)
        self.assertEqual(restored.context()['phase'], 'metric')
        self.assertEqual(restored.inverse, saved['image_error_to_remaining_mm'])
        self.assertIsNone(restored.pending)
        for bad in ([[0,0],[0,0]], [[1,1],[2,2]], [[float('nan'),0],[0,1]], [[1e300,0],[0,1e300]], [[True,0],[0,1]], [[1,0]]):
            with self.subTest(matrix=bad), self.assertRaises(ValueError):
                MetricArmCalibration(dict(saved,image_error_to_remaining_mm=bad))

    def test_clipped_fragment_is_coarse_only_and_bad_probes_stop(self):
        goal = [350,750,650,1000]
        state = MetricArmCalibration()
        moves, done, residual = state.observe(dict(goal=goal,current=[740,600,1000,1000]), [0,150,40], None)
        self.assertEqual(moves['x_mm'], 20)
        self.assertFalse(done)
        self.assertIsNone(state.pending)
        self.assertIsNone(residual)
        for position, current in (([0,150,40],[450,700,750,1000]),
                                  ([2,155,40],[410,705,710,1000]),
                                  ([0,155,42],[410,705,710,1000]),
                                  ([0,155,40],[450,700,750,1000])):
            state = MetricArmCalibration()
            state.observe(dict(goal=goal,current=[450,700,750,1000]), [0,150,40], None)
            with self.subTest(position=position,current=current), self.assertRaises(ValueError):
                state.observe(dict(goal=goal,current=current),position,None)

    def test_robot_rechecks_raw_error_before_accepting_zero_motion(self):
        from robot_control import atomic_json
        for residual, model_aligned, accept in ((dict(x_mm=.7,y_mm=.5),False,True),
                (dict(x_mm=5,y_mm=0),False,False), (dict(x_mm=1,y_mm=-1),False,False),
                (None,False,False)):
            with self.subTest(residual=residual), tempfile.TemporaryDirectory() as directory:
                control = fake_control()
                control.directory = Path(directory)
                control.capture.return_value = dict(image='data:image/jpeg;base64,AQ==')
                control.arm_step = Mock()
                control.arm.get_coords_info.return_value = [0,150,40]
                def respond(path, value):
                    atomic_json(path,value)
                    response = dict(id=value['id'],result=dict(x_mm=0,turn_deg=0),aligned_perfectly=True)
                    if value['stage']=='arm':
                        self.assertEqual(value['arm_position_mm'],[0,150,40])
                        response.update(result=dict(x_mm=0,y_mm=0,turn_deg=0),metric_ready=residual is not None,
                            residual_mm=residual,model_aligned=model_aligned)
                    atomic_json(control.directory/'fetch_vision_response.json',response)
                with patch('robot_control.atomic_json',side_effect=respond):
                    packet=dict(vision_item=dict(calibration_z_mm=40),alignment_protocol='metric-v1')
                    if accept:
                        control.calibrate_fetch(packet,None)
                    else:
                        with self.assertRaisesRegex(ValueError,'contradicts|calibration must finish'):
                            control.calibrate_fetch(packet,None)
                control.arm_step.assert_not_called()


if __name__ == '__main__':
    unittest.main()
