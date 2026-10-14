import { useState, type PointerEvent } from 'react';
import { Button } from './ui';
import type { ObservationPoint, Origin, Stations } from './mapGeometry';

import { commandId, hardwareBlocks, type HardwareState } from './api';
export type { HardwareState } from './api';
export type HeldInput = { mode: string; key: string } | null;

type Props = { stations: Stations; state?: HardwareState; connected: boolean; goal: Origin | null;
  command: (type: string, values?: Record<string, unknown>) => Promise<void>;
  hold: (input: HeldInput) => void; stop: () => void };

export function HardwareControls({ stations, state, connected, goal, command, hold, stop }: Props) {
  const [mode, setMode] = useState('BASE');
  const [name, setName] = useState('');
  const [radius, setRadius] = useState(String(state?.clearance_m ?? .20));
  const [measured, setMeasured] = useState(false);
  const [confirmation, setConfirmation] = useState<{ type: string; message: string; values?: Record<string, unknown> } | null>(null);
  const [confirming, setConfirming] = useState(false);
  const idle = connected && state?.phase === 'idle';
  const review = state?.phase === 'review_grasp';
  const holding = state?.phase === 'verify_grasp';
  const move = (event: PointerEvent<HTMLButtonElement>, key: string) => {
    event.preventDefault(); event.currentTarget.setPointerCapture(event.pointerId);
    const input = { mode, key }; hold(input);
    void command('manual', { mode, input });
  };
  const release = () => { hold(null); stop(); };
  const button = (key: string, label: string) => <Button key={key} disabled={!connected || !state?.motion_available || (!idle && state.phase !== 'manual' && !(state.phase === 'working' && state.status === 'manual')) || (mode === 'ARM' && !state.arm_homed)}
    style={{ touchAction: 'none' }} onPointerDown={e => move(e, key)} onPointerUp={release}
    onPointerCancel={release} onLostPointerCapture={() => hold(null)}
    onKeyDown={e => { if (e.key === ' ' || e.key === 'Enter') { e.preventDefault(); if (!e.repeat) { const input = { mode, key }; hold(input); void command('manual', { mode, input }); } } }}
    onKeyUp={e => { if (e.key === ' ' || e.key === 'Enter') { e.preventDefault(); release(); } }} onBlur={() => hold(null)} aria-label={`Hold ${label}`}>{label}</Button>;
  return <details className="hardware-controls" open>
    <summary>Hardware controls</summary>
    {state?.phase === 'teleop' && <div role="status"><strong>TELEOP controls hardware</strong><p>Item Z and GOAL recording remain available. Exit TELEOP to resume WebUI motion; stopped feedback is verified automatically.</p></div>}
    {confirmation && <div role="alertdialog" aria-label="Confirm hardware action">
      <p>{confirmation.message}</p>
      <Button disabled={confirming || !connected || (!idle && !(state?.phase === 'fault' && confirmation.type === 'recover_stop'))} onClick={async () => {
        setConfirming(true);
        try { await command(confirmation.type, confirmation.values); }
        finally { setConfirming(false); setConfirmation(null); }
      }}>{confirming ? 'Checking…' : 'Confirm'}</Button>
      <Button disabled={confirming} onClick={() => setConfirmation(null)}>Cancel</Button>
    </div>}
    {state?.phase === 'fault' && <div role="alert"><strong>Stop unconfirmed · motion locked</strong>
      <p>{state.status}{state.arm_error && ` · ${state.arm_error}`}</p>
      <Button disabled={confirming || !connected} onClick={() => setConfirmation({ type: 'recover_stop', values: { confirmed: true }, message: 'Have you physically verified that the base, wheels and arm have stopped? Any latched motor overcurrent will be reset with zero motion. Fresh base and arm feedback is required to clear the fault. Homing confirmation will be reset. The previous task will not resume.' })}>Verify stopped and clear fault</Button>
    </div>}
    <span className="muted">{state ? `Driver ${state.driver_watchdog ? '✓' : '✗'} · Sensors ${state.sensors_ready ? '✓' : '✗'} · Exclusive ${state.exclusive ? '✓' : '✗'} · Pose confirmed ${state.localized ? '✓' : '✗'} · Zones ${state.zones_ready ? '✓' : '✗'}` : 'Controller unavailable'}</span>
    {!state?.mapping_mode && <div className="action-line">
      <Button disabled={!idle || !goal || state?.mapping_mode} onClick={() => void command('initial_pose', { goal })}>Set pose from goal</Button>
      <Button disabled={!idle} onClick={() => void command('confirm_localization')}>Confirm map pose</Button>
    </div>}
    <span className="muted">{state?.mapping_mode ? 'Live SLAM establishes the map pose automatically from fresh LiDAR and odometry.' : 'Pose confirmation is a manual check: the map marker must match the real robot position and heading. Confirmations reset when the controller restarts.'}</span>
    <div className="action-line">
      <Button disabled={!idle || !state?.arm_available || !state.arm_homed || !state.transport_angles} onClick={() => void command('transport')}>Transport pose</Button>
      <Button disabled={!idle || !state?.arm_available} onClick={() => setConfirmation({ type: 'home', message: 'Home the real arm now? Clear its full travel area.' })}>Home arm</Button>
      <Button disabled={!idle || !state?.arm_available || state.arm_homed} onClick={() => setConfirmation({ type: 'confirm_homed', message: 'Confirm the arm has already completed homing since power-on?' })}>{state?.arm_homed ? 'Arm homed ✓' : 'Already homed'}</Button>
    </div>
    <span className="muted">P340 {state?.arm_angles?.map(v => `${v.toFixed(1)}°`).join(' / ') || state?.arm_error || 'Feedback unavailable'}</span>
    <label className="setting-row">Manual mode<select value={mode} disabled={!idle} onChange={e => { hold(null); setMode(e.target.value); }}>
      {['BASE', 'ARM', 'PICKUP'].map(v => <option key={v}>{v}</option>)}
    </select></label>
    <span className="muted">Hold with pointer to move. Release to stop. Manual base driving bypasses no-go planning; watch the robot.</span>
    <div className="action-line">{mode !== 'ARM' && [['forward','Forward'],['back','Back'],['ccw','↶'],['cw','↷'], ...(mode === 'BASE' ? [['left','Left'],['right','Right']] : [])].map(([key,label]) => button(key,label))}</div>
    <div className="action-line">{mode !== 'BASE' && ['X+','X-','Y+','Y-','Z+','Z-'].map(key => button(key,key))}</div>
    {mode === 'PICKUP' && <span className="muted">Face forward at J1=90°: X forward/back, Y left/right, Z up/down.</span>}
    <div className="action-line">
      <Button disabled={!idle || !state?.arm_homed} onClick={() => void command('gripper', { value: 100 })}>Open gripper</Button>
      <Button disabled={!idle || !state?.arm_homed} onClick={() => void command('gripper', { value: 0 })}>Close gripper</Button>
    </div>
    <label className="setting-row">Folded clearance radius (m)<input type="number" min=".15" max="1" step=".01" value={radius} disabled={!idle} onChange={e => { setRadius(e.target.value); setMeasured(false); }} /></label>
    <label><input type="checkbox" checked={measured} disabled={!idle} onChange={e => setMeasured(e.target.checked)} /> Arm folded; measured radius encloses base, arm and payload</label>
    <Button disabled={!idle || !state?.arm_homed || !measured} onClick={() => void command('transport_record', { clearance_m: Number(radius), measured })}>Record transport pose</Button>
    <span className="muted">{state?.transport_angles ? `Saved transport: ${state.transport_angles.join(', ')}° · ${state.clearance_m} m` : 'Transport calibration required before Fetch'}</span>
    <label className="setting-row">Station name<input value={name} maxLength={100} disabled={!idle} onChange={e => setName(e.target.value)} /></label>
    <Button disabled={!idle || !state?.arm_homed || !state?.localized || !name.trim()} onClick={() => {
      const previous = stations[name.trim()];
      const values = { name: name.trim(), overwrite: Boolean(previous), expected_station: previous };
      if (previous) setConfirmation({ type: 'teach', values, message: 'Overwrite this station with the current base and arm pose? Linked items will need reconfirmation.' });
      else void command('teach', values);
    }}>Teach current base + arm pose</Button>
    <span className="muted">Existing names require overwrite confirmation. J1 must be 75–105° (forward). Link the station to an item in its photo editor.</span>
    {(review || holding) && <div role="alert">
      <strong>{review ? 'Inspect live arm camera; align before grasp' : 'Inspect live arm camera; verify object held'}</strong>
      {review && <div className="action-line">{['X+', 'X-', 'Y+', 'Y-', 'Z+', 'Z-'].map(key => <Button key={key} onClick={() => void command('align_step', { axis: key[0], direction: key[1] === '+' ? 1 : -1 })}>{key} 1 mm</Button>)}</div>}
      <Button disabled={!connected} onClick={() => void command('confirm', { stage: state?.phase, task_id: state?.task_id })}>{review ? 'Confirm grasp alignment' : 'Confirm possession and return'}</Button>
      <Button variant="danger" onClick={stop}>Abort / STOP</Button>
    </div>}
  </details>;
}

export function MapUpdateControls({ state, connected, goal, command, points, revision, saving, selectedId, onSelect, onSave }:
  Pick<Props, 'state' | 'connected' | 'goal' | 'command'> & {
    points: ObservationPoint[]; revision: number; saving: boolean; selectedId: string | null;
    onSelect: (id: string | null) => void; onSave: (points: ObservationPoint[], selectedId?: string | null) => Promise<void>;
  }) {
  const [minutes, setMinutes] = useState('10');
  const [updating, setUpdating] = useState(false);
  const update = state?.map_update;
  const paused = state?.phase === 'update_paused';
  const idle = connected && state?.phase === 'idle';
  const validUpdate = Number.isFinite(Number(minutes)) && Number(minutes) >= 1 && Number(minutes) <= 60;
  const goals = points.map(({ x_m, y_m, yaw_rad }) => ({ x_m, y_m, yaw_rad }));
  const selected = points.find(p => p.id === selectedId);
  const pointBlockers = !connected ? ['Connect to the controller'] : hardwareBlocks(state);
  const blockers = !connected ? ['Connect to the controller'] : hardwareBlocks(state, false);
  if (!goals.length) blockers.push('Add an observation point');
  if (!validUpdate) blockers.push('Choose an update window from 1 to 60 minutes');
  const cannotPlan = !idle || updating || saving || blockers.length > 0;
  const updateCommand = async (type: string) => {
    setUpdating(true);
    try { await command(type, type === 'update_start' || type === 'update_pause' || type === 'map_refresh'
      ? { task_id: state?.task_id } : type === 'survey'
        ? { observation_point_id: selectedId, observation_revision: revision }
        : { limit_s: Number(minutes) * 60, goals }); }
    finally { setUpdating(false); }
  };
  return <div className="map-updates">
    <div className="action-line">
      <Button title="Publish and save the latest LiDAR map" disabled={!connected || updating || (!idle && !paused)} onClick={() => void updateCommand('map_refresh')}>Refresh LiDAR map</Button>
    </div>
    <div className="action-line">
      <Button variant="default" title={blockers.join('; ')} disabled={!connected || updating || (!paused && cannotPlan)} onClick={() => void updateCommand(paused ? 'update_start' : 'map_update')}>Start item updates</Button>
      <Button disabled={!connected || updating || paused || !update?.active} onClick={() => void updateCommand('update_pause')}>Pause item updates</Button>
    </div>
    <div className="action-line">
      <Button disabled={!selected || !idle || updating || saving || pointBlockers.length > 0}
        title={pointBlockers.join('; ')} onClick={() => void updateCommand('survey')}>Update selected point</Button>
      <Button disabled={!selected || !idle || updating || saving}
        onClick={() => void onSave(points.filter(p => p.id !== selectedId), null)}>Remove selected point</Button>
    </div>
    <details className="control-details"><summary>Observation points · {points.length}{selected && ` · view ${points.indexOf(selected) + 1} selected`}</summary>
      <span className="muted">Points stay on the map and are saved with it. Update the selected point once, or start updates for all saved points. Use Refresh LiDAR map separately to publish map changes.</span>
      {blockers.length > 0 && <p className="muted">{blockers.join(' · ')}</p>}
      {selected && pointBlockers.length > 0 && <p className="muted">{pointBlockers.join(' · ')}</p>}
      <label className="setting-row">Update window (minutes)<input type="number" min="1" max="60" value={minutes} disabled={!idle || updating} onChange={e => setMinutes(e.target.value)} /></label>
      {update?.active && <span className="muted">{Math.floor(update.remaining_s / 60)}:{String(update.remaining_s % 60).padStart(2, '0')} left</span>}
      <div className="action-line">
        <Button disabled={!idle || updating || saving || !goal || points.length >= 20} onClick={() => {
          if (goal) { const point = { ...goal, id: commandId() }; void onSave([...points, point], point.id); }
        }}>Add goal</Button>
        <Button disabled={!idle || updating || saving || !points.length} onClick={() => void onSave([], null)}>Clear points</Button>
      </div>
      <div className="action-line">{points.map((p, i) => <Button key={p.id} aria-pressed={p.id === selectedId}
        onClick={() => onSelect(p.id)}>{i + 1}: ({p.x_m.toFixed(2)}, {p.y_m.toFixed(2)}) · {(p.yaw_rad * 180 / Math.PI).toFixed(0)}°</Button>)}</div>
    </details>
  </div>;
}
