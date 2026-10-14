import { useEffect, useState } from 'react';
import { api } from './api';
import { Button } from './ui';
import type { LogReporter } from './activityLog';

type TurnSize = 's' | 'm' | 'l';
type TurnFields = Record<`turn_${TurnSize}_${'speed_rad_s' | 'duration_s'}`, number>;
type FetchSettings = { revision: number; values: TurnFields & { max_step_mm: number; turn_speed_rad_s: number; turn_duration_scale: number; turn_step_deg: number; prompt: string } };

export function FetchTest(props: {
  connected: boolean; editable: boolean; turnEnabled: boolean; reasons: string[]; itemId: string; indexRevision?: number;
  holdingItemId?: string | null; graspToggleAvailable?: boolean;
  itemName?: string; graspZ?: number; metric?: boolean; onLog: LogReporter; command: (type: string, values: Record<string, unknown>) => Promise<void>; stop: () => void;
}) {
  const [settings, setSettings] = useState<FetchSettings | null>(null);
  const [draft, setDraft] = useState<FetchSettings['values'] | null>(null);
  const [pending, setPending] = useState(false);
  useEffect(() => {
    if (!props.connected) return;
    const controller = new AbortController();
    void api<FetchSettings>('/api/fetch-settings', { signal: controller.signal }).then(value => {
      if (!controller.signal.aborted) { setSettings(value); setDraft(value.values); }
    }).catch(failure => { if (!controller.signal.aborted) props.onLog((failure as Error).message, 'error', 'Calibration'); });
    return () => controller.abort();
  }, [props.connected, props.onLog]);

  async function save(preview?: boolean, turnSize?: 's' | 'm' | 'l') {
    if (!settings || !draft || pending) return;
    setPending(true);
    try {
      const saved = JSON.stringify(draft) === JSON.stringify(settings.values) ? settings : await api<FetchSettings>('/api/fetch-settings', {
        method: 'PUT', body: JSON.stringify({ expected_revision: settings.revision, values: draft }),
      });
      setSettings(saved); setDraft(saved.values);
      if (preview === undefined && !turnSize) props.onLog('Calibration settings saved', 'info', 'Calibration');
      if (turnSize) await props.command('fetch_turn_test', { size: turnSize, fetch_revision: saved.revision });
      if (preview !== undefined) await props.command('fetch_test', {
        item_id: props.itemId, index_revision: props.indexRevision, fetch_revision: saved.revision, preview,
      });
    } catch (failure) { props.onLog((failure as Error).message, 'error', 'Calibration'); }
    finally { setPending(false); }
  }
  async function graspTest() {
    if (pending) return;
    setPending(true);
    try {
      await props.command('grasp_test', { item_id: props.itemId, index_revision: props.indexRevision,
        action: props.holdingItemId ? 'place' : 'grasp' });
    } catch (failure) { props.onLog((failure as Error).message, 'error', 'Calibration'); }
    finally { setPending(false); }
  }
  const disabled = pending || !settings || !draft || props.reasons.length > 0;
  return <section className="fetch-test" aria-label="Grasp calibration test">
    <div className="section-divider" /><h3 className="subheading">GRASP TEST</h3>
    {props.itemName && <span className="muted">{props.itemName}</span>}
    <details className="control-details"><summary>Calibration settings</summary><div className="control-detail-body">
    {draft && <>
      <div className="goal-inputs">
        <label>Arm X/Y step limit (mm; 0 = full correction)<input type="number" min={0} step={0.1} value={draft.max_step_mm}
          disabled={!props.editable || pending} onChange={event => setDraft({ ...draft, max_step_mm: Number(event.target.value) })} /></label>

      </div>
      {(['s', 'm', 'l'] as const).map(size => <div className="goal-inputs" key={size}>
        <label>{size.toUpperCase()} speed (rad/s)<input type="number" min={0.005} max={0.1} step={0.001} value={draft[`turn_${size}_speed_rad_s`]}
          disabled={!props.editable || pending} onChange={event => setDraft({ ...draft, [`turn_${size}_speed_rad_s`]: Number(event.target.value) })} /></label>
        <label>{size.toUpperCase()} duration (s)<input type="number" min={0.05} max={15} step={0.05} value={Number(draft[`turn_${size}_duration_s`].toFixed(3))}
          disabled={!props.editable || pending} onChange={event => setDraft({ ...draft, [`turn_${size}_duration_s`]: Number(event.target.value) })} /></label>
        <Button disabled={!props.connected || !props.editable || !props.turnEnabled || pending || !settings}
          title="Save settings, then rotate right once. No LLM or arm movement."
          onClick={() => void save(undefined, size)}>{size.toUpperCase()} ↻ TEST</Button>
      </div>)}
      <span className="muted">Independent S / M / L speeds and seconds. Rotation stops early at {draft.turn_step_deg / 4}° / {draft.turn_step_deg / 2}° / {draft.turn_step_deg}°. Tests and ALIGN use the same settings.</span>
      <Button variant="danger" disabled={!props.connected} onClick={props.stop}>STOP</Button>
      <label className="photo-field">Calibration prompt<textarea className="fetch-prompt" rows={10} maxLength={8000} value={draft.prompt}
        disabled={!props.editable || pending} onChange={event => setDraft({ ...draft, prompt: event.target.value })} /></label>
      <Button disabled={!props.editable || pending || JSON.stringify(draft) === JSON.stringify(settings?.values)} onClick={() => void save()}>Save settings</Button>
    </>}
    </div></details>
    <details className="control-details"><summary>Alignment guide</summary><div className="control-detail-body">
    <span className="muted">Align accepts J1 75–105° near the object. No map-pose confirmation needed for this local test.</span>
    <span className="muted">Shared with Fetch · original-resolution photos. Alignment and rotation tests save edits first.</span>
    <span className="muted">{props.metric ? 'Stage 1: base coarse alignment. Stage 2: reuse the measured scale for this GOAL and height. A new GOAL needs two short X/Y calibration probes. Apply the full remaining X/Y correction in one observation; 0 removes the optional step limit. Physical workspace checks remain active. Model alignment: planar error <1 mm. Controller alignment: full |X| + |Y| <1.25 mm. Missing or ambiguous targets stop.' : 'Stage 1: base coarse alignment. Stage 2: arm X/Y match GOAL using visible bounds: top within 0.5%, horizontal/bottom within 2%, width within 10%, area within 15%. Missing or ambiguous targets stop.'}</span>
    </div></details>
    <div className="action-line">
      <Button disabled={disabled} title={props.reasons.join('; ')} onClick={() => void save(true)}>PREVIEW</Button>
      <Button variant="default" disabled={disabled} title={props.reasons.join('; ')} onClick={() => void save(false)}>ALIGN</Button>
      <Button disabled={disabled || props.graspZ == null || !props.graspToggleAvailable || Boolean(props.holdingItemId && props.holdingItemId !== props.itemId)}
        title={props.reasons.join('; ') || (!props.graspToggleAvailable ? 'Restart the robot controller.' : props.holdingItemId && props.holdingItemId !== props.itemId ? 'Select the held item before placing it.' : props.graspZ == null ? 'Save the item grasp height first.' : props.holdingItemId ? 'Lower and release the held item, then raise the arm.' : 'Run after ALIGN. Grasp and lift; keep the gripper closed until PLACE.')}
        onClick={() => void graspTest()}>{props.holdingItemId ? 'PLACE' : 'GRASP'}</Button>
      <Button variant="danger" disabled={!props.connected} onClick={props.stop}>STOP</Button>
    </div>
    <details className="control-details"><summary>Grasp sequence</summary><div className="control-detail-body">
    <span className="muted">Preview: one coarse-alignment round, no movement. ALIGN first restores ARM GOAL Z within 1 mm, then runs both stages with Z fixed. Gripper stays fixed. Target above GOAL: Y−; below: Y+. Up to 33 observations; no cumulative arm travel cap.</span>
    <span className="muted">{props.graspZ == null ? 'Record the item grasp Z first.' : `Saved grasp Z ${props.graspZ.toFixed(1)} mm.`} First click: lower, clamp, lift and hold. Click PLACE separately to lower, release and raise. X/Y stay fixed.</span>
    </div></details>
    {props.reasons.length > 0 && <details className="control-details"><summary>Test requirements</summary><p className="muted">{props.reasons.join('; ')}</p></details>}
  </section>;
}
