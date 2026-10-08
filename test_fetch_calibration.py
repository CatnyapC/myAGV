import array
from contextlib import nullcontext
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import AsyncMock, Mock, call, patch

from fetch_calibration import alignment_command, calibration_config, pickup_delta, vision_image
from test_robot_control import fake_control
from web_backend.resolve import locate_pickup
from web_backend.storage import atomic_json


class FetchCalibrationTest(unittest.IsolatedAsyncioTestCase):
    def test_mapping_limits_validation_and_compression(self):
        self.assertEqual(alignment_command({'x_mm': 2, 'y_mm': 1}, None), {'X': 2, 'Y': 1})
        self.assertEqual(alignment_command({'x_mm': -2, 'y_mm': -1}, None), {'X': -2, 'Y': -1})
        self.assertEqual(alignment_command({'x_mm': 0, 'y_mm': 0}, None), {'X': 0, 'Y': 0})
        self.assertEqual(alignment_command({'x_mm': 2, 'y_mm': -2}, {'max_step_mm': 1}), {'X': 1, 'Y': -1})
        for bad in (None, {}, {'x_mm': None, 'y_mm': 0}, {'x_mm': True, 'y_mm': 0},
                    {'x_mm': float('nan'), 'y_mm': 0}, {'x_mm': 2.1, 'y_mm': 0},
                    {'x_mm': .05, 'y_mm': 0}, {'x_mm': 0, 'y_mm': 0, 'grip': True}):
            with self.assertRaises(ValueError):
                alignment_command(bad, None)
        with self.assertRaises(ValueError):
            calibration_config({'max_step_mm': 3})
        self.assertEqual(pickup_delta('X', 1), (0, 1, 0))
        self.assertEqual(pickup_delta('Y', 1), (-1, 0, 0))
        cv2 = SimpleNamespace(INTER_AREA=3, IMWRITE_JPEG_QUALITY=1,
            resize=Mock(return_value='small'), imencode=Mock(return_value=(True, array.array('B', [1, 2]))))
        self.assertTrue(vision_image(SimpleNamespace(shape=(480, 640, 3)), cv2).startswith('data:image/jpeg;base64,'))
        self.assertEqual(cv2.resize.call_args.args[1], (480, 360))
        cv2.imencode.assert_called_once_with('.jpg', 'small', [1, 60])
        c = fake_control()
        c.arm.get_coords_info.side_effect = [[0, 200, 0], [0, 201, 0]]
        c.arm.get_angles_info.return_value = [90, 0, 0]
        with patch('robot_control.arm_deadline', return_value=nullcontext()):
            c.arm_step('X', 1, pickup=True)
        c.arm.set_coords.assert_called_once_with([0, 201, 0], 30)

    async def test_minimal_provider_request(self):
        images = dict(front='data:image/jpeg;base64,AQ==', arm='data:image/jpeg;base64,Ag==')
        provider = AsyncMock(return_value={'x_mm': 0, 'y_mm': 0})
        history = [dict(images=images, commanded_mm=dict(X=1, Y=-1))] * 2
        with patch('web_backend.resolve.request_json', provider):
            await locate_pickup(images, dict(name='cup', appearance='red'), 'fake-key', images, history)
        body = provider.call_args.args[0]
        self.assertEqual(body['reasoning'], {'enabled': False})
        self.assertEqual(body['max_tokens'], 64)
        self.assertTrue(body['response_format']['json_schema']['strict'])
        self.assertEqual([part['image_url']['detail'] for part in body['messages'][1]['content']
                          if part['type'] == 'image_url'], ['low'] * 8)
        labels = [part['text'] for part in body['messages'][1]['content'] if part['type'] == 'text']
        self.assertTrue(any('GOAL' in label for label in labels))
        self.assertTrue(any('"X": 1' in label and '"Y": -1' in label for label in labels))

    def test_closed_loop_recaptures_and_missing_target_stops(self):
        c = fake_control()
        c.task_id, c.arm_step = 'task', Mock()
        c.capture.return_value = {'image': 'compressed'}
        with tempfile.TemporaryDirectory() as folder:
            c.directory = Path(folder)
            results = iter([{'x_mm': 2, 'y_mm': 2}, {'x_mm': 0, 'y_mm': 0}])
            requests = []

            def respond(path, value):
                requests.append(value)
                atomic_json(path, value)
                atomic_json(c.directory / 'fetch_vision_response.json', {'id': value['id'], 'result': next(results)})

            with patch('robot_control.atomic_json', side_effect=respond):
                c.calibrate_fetch({'vision_item': {'name': 'cup'}}, None)
            self.assertEqual(c.capture.call_args_list, [call('front', vision=True), call('arm', vision=True)] * 2)
            self.assertEqual([entry.args[0] for entry in c.arm_step.call_args_list], ['X', 'X', 'Y', 'Y'])
            self.assertEqual(requests[1]['history'][0]['commanded_mm'], {'X': 2, 'Y': 2})
            self.assertFalse(list(c.directory.iterdir()))
            c.arm_step.reset_mock()
            results = iter([{'x_mm': None, 'y_mm': 0}])
            with patch('robot_control.atomic_json', side_effect=respond), self.assertRaises(ValueError):
                c.calibrate_fetch({'vision_item': {'name': 'cup'}}, None)
            c.arm_step.assert_not_called()
            self.assertFalse(list(c.directory.iterdir()))
            requests.clear()
            results = iter([{'x_mm': .5, 'y_mm': 0}] * 8 + [{'x_mm': 0, 'y_mm': 0}])
            with patch('robot_control.atomic_json', side_effect=respond):
                c.calibrate_fetch({'vision_item': {'name': 'cup'}}, None)
            self.assertEqual(len(requests), 9)
            self.assertEqual(c.arm_step.call_count, 8)
            self.assertEqual(len(requests[-1]['history']), 2)


if __name__ == '__main__':
    unittest.main()
