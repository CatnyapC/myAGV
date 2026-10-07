import assert from 'node:assert/strict';
import { test } from 'node:test';
import { DEFAULT_LAYOUT, canonicalLayout, dividers, moveDivider, validatedLayout, viewportCellHeight } from './layout.ts';

test('viewport fill, coupled divider resizing, minimums and reset', () => {
  const original = validatedLayout(DEFAULT_LAYOUT);
  const height = viewportCellHeight(original, 840);
  assert.equal(height * 12, 840);
  const seams = dividers(original);
  assert.equal(seams.length, 3);
  const mapEdge = seams.find(s => s.axis === 'x' && s.position === 12)!;
  assert.deepEqual(mapEdge.after, ['arm', 'front']);
  const moved = moveDivider(original, mapEdge, 2, 80, height);
  assert.equal(moved.find(t => t.id === 'map')!.w, 14);
  assert.equal(moved.find(t => t.id === 'operate')!.x, 18);
  for (const id of ['front', 'arm']) { const tile = moved.find(t => t.id === id)!; assert.equal(tile.x, 14); assert.equal(tile.w, 4); }
  const cameraEdge = seams.find(s => s.axis === 'y')!;
  const taller = moveDivider(original, cameraEdge, 1, 80, height);
  assert.equal(taller.find(t => t.id === 'front')!.h, 7);
  assert.equal(taller.find(t => t.id === 'arm')!.y, 7);
  assert.equal(taller.find(t => t.id === 'arm')!.h, 5);
  const clamped = moveDivider(original, mapEdge, 100, 80, height);
  assert.ok(clamped.find(t => t.id === 'front')!.w * 80 >= 288);
  assert.equal(clamped.reduce((area, t) => area + t.w * t.h, 0), 24 * 12);
  assert.deepEqual(validatedLayout(JSON.parse(JSON.stringify(moved))), moved);
  assert.deepEqual(validatedLayout([{ id: 'unknown' }]), original);
  assert.ok(viewportCellHeight(original, 250) * 12 >= 452); // Short windows scroll rather than crush panels.
  assert.deepEqual(validatedLayout(DEFAULT_LAYOUT), original); // Reset ignores altered saved proportions.
  assert.deepEqual(canonicalLayout(original, original, moved, 24), moved);
  const narrow = original.map(t => ({ ...t, x: t.x / 2, w: t.w / 2 }));
  const narrowMoved = narrow.map(t => t.id === 'front' ? { ...t, w: t.w + 1 } : t);
  const saved = canonicalLayout(original, narrow, narrowMoved, 12);
  assert.equal(saved.find(t => t.id === 'front')!.w, 8);
  assert.deepEqual(saved.find(t => t.id === 'map'), original.find(t => t.id === 'map'));
  const phone = original.map((t, i) => ({ ...t, x: 0, y: i * 12, w: 1 }));
  const phoneMoved = phone.map(t => t.id === 'front' ? { ...t, h: t.h + 1 } : t);
  assert.equal(canonicalLayout(original, phone, phoneMoved, 1).find(t => t.id === 'front')!.w, 6);
});
