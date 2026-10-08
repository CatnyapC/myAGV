import assert from 'node:assert/strict';
import test from 'node:test';
import { loadPhotoSelection, savePhotoSelection } from './photoSelection.ts';

test('selection survives reload, clearing, invalid storage and unavailable storage', () => {
  const original = Object.getOwnPropertyDescriptor(globalThis, 'localStorage');
  let stored: string | null = null;
  Object.defineProperty(globalThis, 'localStorage', { configurable: true, value: {
    getItem: () => stored,
    setItem: (_key: string, value: string) => { stored = value; },
  } });
  try {
    assert.deepEqual(loadPhotoSelection(), { itemId: '', photoId: null });
    savePhotoSelection('cup', 'cup-observation');
    assert.deepEqual(loadPhotoSelection(), { itemId: 'cup', photoId: 'cup-observation' });
    savePhotoSelection('bottle', 'bottle-reference');
    assert.deepEqual(loadPhotoSelection(), { itemId: 'bottle', photoId: 'bottle-reference' });
    savePhotoSelection('', null);
    assert.deepEqual(loadPhotoSelection(), { itemId: '', photoId: null });
    for (const invalid of ['broken', 'null', '[]', '{"itemId":3,"photoId":null}', '{"itemId":"cup","photoId":false}']) {
      stored = invalid;
      assert.deepEqual(loadPhotoSelection(), { itemId: '', photoId: null });
    }
    Object.defineProperty(globalThis, 'localStorage', { configurable: true, get() { throw new Error('Storage blocked'); } });
    assert.deepEqual(loadPhotoSelection(), { itemId: '', photoId: null });
    assert.doesNotThrow(() => savePhotoSelection('cup', null));
  } finally {
    if (original) Object.defineProperty(globalThis, 'localStorage', original);
    else Reflect.deleteProperty(globalThis, 'localStorage');
  }
});
