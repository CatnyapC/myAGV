import assert from 'node:assert/strict';
import test from 'node:test';
import { commandId, hardwareBlocks, waitForControlLease, type HardwareState } from './api.ts';

test('commands wait for a fresh receipt, and STOP or hidden page cancels pending commands', async () => {
  let state = { ack: 1, epoch: 0, visible: true };
  let ready = false;
  const waiting = waitForControlLease(() => state).then(() => { ready = true; });
  await new Promise(resolve => setTimeout(resolve, 30));
  assert.equal(ready, false);
  state.ack++;
  await waiting;
  const stopped = waitForControlLease(() => state);
  state.epoch++;
  await assert.rejects(stopped, /cancelled/);
  state.visible = false;
  await assert.rejects(waitForControlLease(() => state), /cancelled/);
  state = { ack: 0, epoch: 0, visible: true };
  await assert.rejects(waitForControlLease(() => state), /heartbeat unavailable/);
});

test('hardware action locks explain calibration, faults and unavailable feedback', () => {
  const state: HardwareState = { stop_epoch: 0, phase: 'idle', status: 'Ready', arm_available: true,
    arm_homed: true, navigation_ready: true, motion_available: true, localized: true,
    driver_watchdog: true, exclusive: true, sensors_ready: true, transport_angles: [0,10,20],
    clearance_m: .25, zones_ready: true, planner_footprint_ready: true, stop_confirmed: true };
  assert.deepEqual(hardwareBlocks(state), []);
  assert.deepEqual(hardwareBlocks({ ...state, phase: 'teleop' }), ['Exit TELEOP, then verify stopped hardware to resume WebUI control']);
  for (const arm_available of [false, true]) {
    const baseOnly = { ...state, arm_available, arm_homed: false, transport_angles: undefined };
    assert.deepEqual(hardwareBlocks(baseOnly, false), []);
    assert.ok(hardwareBlocks(baseOnly).length > 0);
    assert.ok(hardwareBlocks({ ...baseOnly, localized: false }, false).some(s => s.includes('robot position')));
    assert.ok(hardwareBlocks({ ...baseOnly, planner_footprint_ready: false }, false).some(s => s.includes('planner footprint')));
    assert.ok(hardwareBlocks({ ...baseOnly, navigation_ready: false }, false).includes('Controller not ready'));
  }
  const missing = hardwareBlocks({ ...state, navigation_ready: false, arm_homed: false, transport_angles: undefined });
  assert.ok(missing.some(s => s.includes('Home arm')));
  assert.ok(missing.some(s => s.includes('measured clearance')));
  assert.ok(!missing.some(s => s.includes('robot position')));
  assert.ok(hardwareBlocks({ ...state, localized: false }).some(s => s.includes('robot position')));
  assert.ok(hardwareBlocks({ ...state, phase: 'fault' }).some(s => s.includes('clear the fault')));
  assert.deepEqual(hardwareBlocks(), ['Controller unavailable']);
  assert.match(commandId(), /^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/);
});
