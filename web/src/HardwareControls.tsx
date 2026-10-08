import { useState, type PointerEvent } from 'react';
import { Button } from './ui';
import type { Origin, Stations } from './mapGeometry';

import { hardwareBlocks, type HardwareState } from './api';
export type { HardwareState } from './api';
export type HeldInput = { mode: string; key: string } | null;

type Props = { stations: Stations; state?: HardwareState; connected: boolean; goal: Origin | null;
  command: (type: string, values?: Record<string, unknown>) => Promise<void>;
  hold: (input: HeldInput) => void; stop: () => void };

export function HardwareControls({ stations, state, connected, goal, command, hold, stop }: Props) {
  const [mode, setMode] = useState('BASE');
  const [name, setName] = useState('');
  const [radius, setRadius] = useState('0.25');
  const [measured, setMeasured] = useState(false);
  const [survey, setSurvey] = useState<Origin[]>([]);
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
      <Button disabled={confirming || !connected} onClick={() => setConfirmation({ type: 'recover_stop', values: { confirmed: true }, message: 'Have you physically verified that the base and arm have stopped? Fresh base and arm feedback is required to clear the fault. Homing confirmation will be reset.' })}>Verify stopped and clear fault</Button>
    </div>}
    <span className="muted">{state ? `Driver ${state.driver_watchdog ? '✓' : '✗'} · Sensors ${state.sensors_ready ? '✓' : '✗'} · Exclusive ${state.exclusive ? '✓' : '✗'} · Pose confirmed ${state.localized ? '✓' : '✗'} · Zones ${state.zones_ready ? '✓' : '✗'}` : 'Controller unavailable'}</span>
    <div className="action-line">
      <Button disabled={!idle || !goal} onClick={() => void command('initial_pose', { goal })}>Set pose from goal</Button>
      <Button disabled={!idle} onClick={() => void command('confirm_localization')}>Confirm map pose</Button>
    </div>
    <span className="muted">Pose confirmation is a manual check: the map marker must match the real robot position and heading. Confirmations reset when the controller restarts.</span>
    <div className="action-line">
      <Button disabled={!idle || state?.arm_available} onClick={() => void command('connect_arm')}>Connect P340</Button>
      <Button disabled={!idle || !state?.arm_available} onClick={() => setConfirmation({ type: 'home', message: 'Home the real arm now? Clear its full travel area.' })}>Home arm</Button>
      <Button disabled={!idle || !state?.arm_available} onClick={() => setConfirmation({ type: 'confirm_homed', message: 'Confirm the arm has already completed homing since power-on?' })}>Already homed</Button>
    </div>
    <span className="muted">P340 {state?.arm_angles?.map(v => `${v.toFixed(1)}°`).join(' / ') || state?.arm_error || 'Feedback unavailable'}</span>
    <label className="setting-row">Manual mode<select value={mode} disabled={!idle} onChange={e => { hold(null); setMode(e.target.value); }}>
      {['BASE', 'ARM', 'PICKUP'].map(v => <option key={v}>{v}</option>)}
    </select></label>
    <span className="muted">Hold with pointer to move. Release to stop. Manual base driving bypasses no-go planning; watch the robot.</span>
    <div className="action-line">{mode !== 'ARM' && [['forward','Forward'],['back','Back'],['ccw','↶'],['cw','↷'], ...(mode === 'BASE' ? [['left','Left'],['right','Right']] : [])].map(([key,label]) => button(key,label))}</div>
    <div className="action-line">{mode !== 'BASE' && ['X+','X-', ...(mode === 'ARM' ? ['Y+','Y-'] : []),'Z+','Z-'].map(key => button(key,key))}</div>
    <div className="action-line">
      <Button disabled={!idle || !state?.arm_homed} onClick={() => void command('gripper', { value: 100 })}>Open gripper</Button>
      <Button disabled={!idle || !state?.arm_homed} onClick={() => void command('gripper', { value: 0 })}>Close gripper</Button>
    </div>
    <label className="setting-row">Folded clearance radius (m)<input type="number" min=".15" max="1" step=".01" value={radius} disabled={!idle} onChange={e => { setRadius(e.target.value); setMeasured(false); }} /></label>
    <label><input type="checkbox" checked={measured} disabled={!idle} onChange={e => setMeasured(e.target.checked)} /> Arm folded, J1=0; measured radius encloses base, arm and payload</label>
    <Button disabled={!idle || !state?.arm_homed || !measured} onClick={() => void command('transport_record', { clearance_m: Number(radius), measured })}>Record transport pose</Button>
    <span className="muted">{state?.transport_angles ? `Saved transport: ${state.transport_angles.join(', ')}° · ${state.clearance_m} m` : 'Transport calibration required before Fetch or survey'}</span>
    <label className="setting-row">Station name<input value={name} maxLength={100} disabled={!idle} onChange={e => setName(e.target.value)} /></label>
    <Button disabled={!idle || !state?.arm_homed || !state?.localized || !name.trim()} onClick={() => {
      const previous = stations[name.trim()];
      const values = { name: name.trim(), overwrite: Boolean(previous), expected_station: previous };
      if (previous) setConfirmation({ type: 'teach', values, message: 'Overwrite this station with the current base and arm pose? Linked items will need reconfirmation.' });
      else void command('teach', values);
    }}>Teach current base + arm pose</Button>
    <span className="muted">Existing names require overwrite confirmation. J1 must be zero. Link the station to an item in its photo editor.</span>
    {(review || holding) && <div role="alert">
      <strong>{review ? 'Inspect live arm camera; align before grasp' : 'Inspect live arm camera; verify object held'}</strong>
      {review && <div className="action-line">{['X+', 'X-', 'Z+', 'Z-'].map(key => <Button key={key} onClick={() => void command('align_step', { axis: key[0], direction: key[1] === '+' ? 1 : -1 })}>{key} 1 mm</Button>)}</div>}
      <Button disabled={!connected} onClick={() => void command('confirm', { stage: state?.phase, task_id: state?.task_id })}>{review ? 'Confirm grasp alignment' : 'Confirm possession and return'}</Button>
      <Button variant="danger" onClick={stop}>Abort / STOP</Button>
    </div>}
    <details><summary>Survey · {survey.length}/20 viewing poses</summary>
      <div className="action-line">
        <Button disabled={!idle || !goal || survey.length >= 20} onClick={() => goal && setSurvey(old => [...old, goal])}>Add goal</Button>
        <Button disabled={!idle} onClick={() => setSurvey([])}>Clear queue</Button>
        <Button disabled={!idle || hardwareBlocks(state).length > 0 || !survey.length} onClick={() => void command('survey', { goals: survey })}>Run survey</Button>
      </div><span className="muted">{survey.map((p,i) => `${i+1}: (${p.x_m.toFixed(2)}, ${p.y_m.toFixed(2)})`).join(' · ')}</span>
    </details>
  </details>;
}
