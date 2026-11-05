import math
from types import SimpleNamespace
import unittest

from obstacle_filter import filter_grid, filter_scan


class ObstacleFilterTest(unittest.TestCase):
    def test_small_clusters_removed_walls_unknown_and_original_retained(self):
        grid = dict(width=10, height=5, resolution_m=.05, cells=[0]*50)
        grid['cells'][0] = -1
        grid['cells'][12] = grid['cells'][13] = 100
        for i in range(30, 40):
            grid['cells'][i] = 100
        filtered = filter_grid(grid, .15)
        self.assertEqual(filtered['cells'][12:14], [0, 0])
        self.assertEqual(filtered['cells'][30:40], [100]*10)
        self.assertEqual(filtered['cells'][0], -1)
        self.assertEqual(filter_grid(filtered, 0)['cells'], grid['cells'])
        scan = SimpleNamespace(ranges=[5.36]*12 + [1.589, 1.594] + [2.83]*12,
                               range_min=.1, range_max=12, angle_min=0, angle_increment=.0137)
        clean = filter_scan(scan, .15)
        self.assertTrue(all(math.isinf(v) for v in clean[12:14]))
        self.assertEqual(clean[:12], scan.ranges[:12])
        self.assertEqual(clean[14:], scan.ranges[14:])
        self.assertEqual(filter_scan(scan, 0), scan.ranges)


if __name__ == '__main__':
    unittest.main()
