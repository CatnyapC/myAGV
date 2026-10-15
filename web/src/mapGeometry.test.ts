import assert from 'node:assert/strict';
import { test } from 'node:test';
import { chassisOutline, latestMapObservation, mapFitBounds, rectangleCorners, toView, toWorld, type MapInfo, type Navigation, type Photo } from './mapGeometry.ts';

test('map shows only the latest current observation across cameras and items', () => {
  const older: Extract<Photo, { kind: 'observation' }> = { id: 'older', kind: 'observation',
    saved_at_s: 100, captured_at_s: 1, camera_id: 'front', map_id: 'room', map_revision: 1,
    frame: 'map', base_pose: { x_m: 1, y_m: 2, yaw_rad: 0 }, source: 'ros',
    image_url: '', available: true, map_matches: true, item_id: 'cup' };
  const newest = { ...older, id: 'newest', camera_id: 'arm' as const, item_id: 'box', captured_at_s: 3 };
  const photos: Photo[] = [newest, { ...older, id: 'middle', captured_at_s: 2 }, older,
    { ...newest, id: 'historical', captured_at_s: 4, current: false },
    { ...newest, id: 'other-map', captured_at_s: 5, map_id: 'elsewhere' },
    { ...older, id: 'reference', kind: 'reference' }];
  assert.equal(latestMapObservation(photos, 'room'), newest);
  assert.equal(photos.length, 6);
  const tied = { ...newest, id: 'tied' };
  assert.equal(latestMapObservation([...photos, tied], 'room'), tied);
  assert.equal(latestMapObservation([], 'room'), undefined);
  assert.equal(latestMapObservation(photos, 'missing'), undefined);
  assert.equal(latestMapObservation([{ ...older, current: false }], 'room'), undefined);
});

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

test('fit centers robot near the top edge and includes the full explored region', () => {
  for (const yaw_rad of [0, .6632251158]) {
    const origin = { x_m: -13.8, y_m: -15.4, yaw_rad };
    const info: MapInfo = { map_id: 'room', revision: 3, frame: 'map', width: 544, height: 320,
      resolution_m: .05, origin, png_url: '', display: { width_m: 27.2, height_m: 16, origin,
        view_revision: 5, alignment_available: true, content_bounds: [[10.5, 9.9], [15.75, 16.65]] } };
    const [x_m, y_m] = toWorld([15.22, 13.84], origin);
    const navigation: Navigation = { map_id: info.map_id, frame: 'map', source: 'ros', stamp_s: 1,
      pose: { x_m, y_m, yaw_rad: -.01 }, goal: null, path: [], moving: false, phase: 'idle', status: '',
      footprint: { length_m: .32, width_m: .28 }, costmap: { ready: true, applied_zone_revision: 0, clearance_m: .2 } };
    const bounds = mapFitBounds(info, navigation);
    const center = toView([x_m, y_m], origin);
    center.forEach((value, axis) => assert.ok(Math.abs((bounds[0][axis] + bounds[1][axis]) / 2 - value) < 1e-9));
    for (const point of [...info.display.content_bounds!, ...chassisOutline(navigation.pose!, .32, .28).map(p => toView(p, origin))]) {
      point.forEach((value, axis) => assert.ok(value >= bounds[0][axis] && value <= bounds[1][axis]));
    }
    assert.deepEqual(mapFitBounds(info, null), info.display.content_bounds);
    assert.deepEqual(mapFitBounds(info, { ...navigation, pose: null }), info.display.content_bounds);
    assert.deepEqual(mapFitBounds(info, { ...navigation, map_id: 'old-map' }), info.display.content_bounds);
    const legacy = { ...info, display: { ...info.display, content_bounds: undefined } };
    assert.deepEqual(mapFitBounds(legacy, null), [[0, 0], [16, 27.2]]);
  }
});
