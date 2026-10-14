import assert from 'node:assert/strict';
import test from 'node:test';
import { appendLog, type LogEntry } from './activityLog.ts';

test('log retains messages across actions and groups repeated failures without losing severity or source', () => {
  const initial = appendLog([], 'Connection lost', 'error', 'Connection', 1000);
  assert.equal(appendLog(initial, ' '), initial);
  const repeated = appendLog(initial, 'Connection lost', 'error', 'Connection', 2000);
  assert.equal(repeated.length, 1);
  assert.equal(repeated[0].repeats, 2);
  assert.equal(repeated[0].time, 2000);
  assert.equal(initial[0].repeats, 1);
  const connected = appendLog(repeated, 'Connected', 'info', 'Connection', 3000);
  assert.equal(connected[0].message, 'Connection lost');
  assert.deepEqual(connected.map(entry => entry.id), [1, 2]);
  assert.equal(appendLog(initial, 'Connection lost', 'error', 'Connection', 6000).length, 2);
  assert.equal(appendLog(initial, 'Connection lost', 'info', 'Connection', 2000).length, 2);
  assert.equal(appendLog(initial, 'Connection lost', 'error', 'Task', 2000).length, 2);
});

test('log bounds memory while keeping latest messages in order', () => {
  let entries: LogEntry[] = [];
  for (let i = 0; i < 205; i++) entries = appendLog(entries, `Message ${i}`, 'info', 'Task', i);
  assert.equal(entries.length, 200);
  assert.equal(entries[0].message, 'Message 5');
  assert.equal(entries.at(-1)?.message, 'Message 204');
  assert.equal(new Set(entries.map(entry => entry.id)).size, 200);
});
