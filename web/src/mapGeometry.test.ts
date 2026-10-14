import assert from 'node:assert/strict';
import { test } from 'node:test';
import { chassisOutline, rectangleCorners, toView, toWorld } from './mapGeometry.ts';

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
  const pose = { x_m: 3, y_m: 4, yaw_rad: Math.PI / 2 };
  const chassis = chassisOutline(pose, .32, .28);
  assert.equal(chassis.length, 5);
  const xs = chassis.map(p => p[0]), ys = chassis.map(p => p[1]);
  assert.ok(Math.abs(Math.max(...xs) - Math.min(...xs) - .28) < 1e-9);
  assert.ok(Math.abs(Math.max(...ys) - Math.min(...ys) - .32) < 1e-9);
});

test('pentagon nose follows base forward through pose and display rotation', () => {
  for (const yaw_rad of [0, Math.PI / 2, Math.PI, -.7]) {
    const pose = { x_m: 3, y_m: 4, yaw_rad };
    const origin = { x_m: -1, y_m: 2, yaw_rad: .6 };
    const outline = chassisOutline(pose, .32, .28);
    const local = outline.map(point => toView(toWorld(toView(point, origin), origin), pose));
    const expected = [[-.14, -.16], [-.14, .08], [0, .16], [.14, .08], [.14, -.16]];
    local.forEach((point, i) => point.forEach((value, axis) => assert.ok(Math.abs(value - expected[i][axis]) < 1e-9)));
  }
});
