import assert from 'node:assert/strict';
import { test } from 'node:test';
import { rectangleCorners, toView, toWorld } from './mapGeometry.ts';

test('rotated display coordinates preserve world zones and reverse input once', () => {
  const origin = { x_m: -3.2, y_m: 1.4, yaw_rad: 0.6632251158 };
  const point: [number, number] = [2.45, -1.15];
  const result = toWorld(toView(point, origin), origin);
  result.forEach((v, i) => assert.ok(Math.abs(v - point[i]) < 1e-9));
  const corners = rectangleCorners([4, 3], [1, 1], origin);
  assert.deepEqual(corners.map(p => toView(p, origin)).map(p => p.map(v => Math.round(v))), [[1, 1], [1, 3], [4, 3], [4, 1]]);
  const next = { x_m: 7, y_m: -2, yaw_rad: -0.45 };
  corners.forEach(p => toWorld(toView(p, next), next).forEach((v, i) => assert.ok(Math.abs(v - p[i]) < 1e-9)));
  const zero = { x_m: 0, y_m: 0, yaw_rad: 0 };
  assert.deepEqual(toView([2, 5], zero), [5, 2]);
});
