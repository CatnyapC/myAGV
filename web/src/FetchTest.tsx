import { useEffect, useRef, useState } from 'react';
import { api } from './api';
import { Button } from './ui';

type FetchSettings = { revision: number; values: { max_step_mm: number; max_total_mm: number; turn_speed_rad_s: number; turn_duration_scale: number; turn_step_deg: number; prompt: string } };
type Part = { type: string; text?: string; image_url?: { url: string } };
type Round = {
  id: string; round: number; mode?: 'grasp' | 'approach'; preview: boolean; status: string; elapsed_s?: number; started_at_s: number;
  model?: string; messages?: [{ content: string }, { content: Part[] }];
  result_json?: string; correction?: { X: number; turn_deg: number }; error?: string;
  base_command?: { forward_m: number; turn_deg: number }; corrected_location?: { x_m: number; y_m: number; uncertainty_m: number };
};

export function FetchTest(props: {
  connected: boolean; editable: boolean; turnEnabled: boolean; reasons: string[]; itemId: string; indexRevision?: number;
  itemName?: string; status?: string; command: (type: string, values: Record<string, unknown>) => Promise<void>; stop: () => void;
}) {
  const [settings, setSettings] = useState<FetchSettings | null>(null);
  const [draft, setDraft] = useState<FetchSettings['values'] | null>(null);
  const [rounds, setRounds] = useState<Round[]>([]);
  const [pending, setPending] = useState(false);
  const [error, setError] = useState('');
  const logRevision = useRef(-1);
  useEffect(() => {
    if (!props.connected) return;
    const controller = new AbortController();
    void api<FetchSettings>('/api/fetch-settings', { signal: controller.signal }).then(value => {
      if (!controller.signal.aborted) { setSettings(value); setDraft(value.values); }
    }).catch(failure => { if (!controller.signal.aborted) setError((failure as Error).message); });
    return () => controller.abort();
  }, [props.connected]);
  useEffect(() => {
    if (!props.connected) return;
    const controller = new AbortController();
    let timer: ReturnType<typeof setTimeout>;
    async function poll() {
      try {
        const value = await api<{ revision: number; rounds?: Round[] }>(`/api/fetch-log?after=${logRevision.current}`, { signal: controller.signal });
        if (!controller.signal.aborted) {
          logRevision.current = value.revision;
          if (value.rounds) setRounds(value.rounds);
        }
      } catch (failure) { if (!controller.signal.aborted) setError((failure as Error).message); }
      if (!controller.signal.aborted) timer = setTimeout(() => void poll(), 1000);
    }
    // Reconnect may reach a restarted API with the same revision but no prior logs.
    logRevision.current = -1;
    void poll();
    return () => { controller.abort(); clearTimeout(timer); };
  }, [props.connected]);

  async function save(preview?: boolean, turnSize?: 's' | 'm' | 'l') {
    if (!settings || !draft || pending) return;
    setPending(true); setError('');
    try {
      const saved = JSON.stringify(draft) === JSON.stringify(settings.values) ? settings : await api<FetchSettings>('/api/fetch-settings', {
        method: 'PUT', body: JSON.stringify({ expected_revision: settings.revision, values: draft }),
      });
      setSettings(saved); setDraft(saved.values);
      if (turnSize) await props.command('fetch_turn_test', { size: turnSize, fetch_revision: saved.revision });
      if (preview !== undefined) await props.command('fetch_test', {
        item_id: props.itemId, index_revision: props.indexRevision, fetch_revision: saved.revision, preview,
      });
    } catch (failure) { setError((failure as Error).message); }
    finally { setPending(false); }
  }
  const disabled = pending || !settings || !draft || props.reasons.length > 0;
  return <section className="fetch-test" aria-label="Grasp calibration test">
    <div className="section-divider" /><h3 className="subheading">GRASP TEST</h3>
    <span className="muted">Item: {props.itemName || 'Select an item above'} · Align accepts J1 75–105° near the object.</span>
    {draft && <>
      <div className="goal-inputs">
        <label>Arm X step (mm)<input type="number" min={0.1} max={10} step={0.1} value={draft.max_step_mm}
          disabled={!props.editable || pending} onChange={event => setDraft({ ...draft, max_step_mm: Number(event.target.value) })} /></label>
        <label>Total arm X (mm)<input type="number" min={draft.max_step_mm} max={30} step={1} value={draft.max_total_mm}
          disabled={!props.editable || pending} onChange={event => setDraft({ ...draft, max_total_mm: Number(event.target.value) })} /></label>
      </div>
      <div className="goal-inputs">
        <label>Turn speed (rad/s)<input type="number" min={0.005} max={0.1} step={0.001} value={draft.turn_speed_rad_s}
          disabled={!props.editable || pending} onChange={event => setDraft({ ...draft, turn_speed_rad_s: Number(event.target.value) })} /></label>
        <label>Turn duration (%)<input type="number" min={10} max={150} step={5} value={Math.round(draft.turn_duration_scale * 100)}
          disabled={!props.editable || pending} onChange={event => setDraft({ ...draft, turn_duration_scale: Number(event.target.value) / 100 })} /></label>
        <label>Large turn (°)<input type="number" min={0.4} max={2} step={0.1} value={draft.turn_step_deg}
          disabled={!props.editable || pending} onChange={event => setDraft({ ...draft, turn_step_deg: Number(event.target.value) })} /></label>
      </div>
      <span className="muted">Base large / medium / small: {draft.turn_step_deg}° / {draft.turn_step_deg / 2}° / {draft.turn_step_deg / 4}°. Large pulse ≈{(draft.turn_duration_scale * draft.turn_step_deg * Math.PI / 180 / draft.turn_speed_rad_s).toFixed(2)} s. Save or press ALIGN to apply next run.</span>
      <div className="action-line">{(['s', 'm', 'l'] as const).map(size => <Button key={size}
        disabled={!props.connected || !props.editable || !props.turnEnabled || pending || !settings}
        title="Save settings, then rotate right once. No LLM or arm movement."
        onClick={() => void save(undefined, size)}>{size.toUpperCase()} ↻</Button>)}
        <Button variant="danger" disabled={!props.connected} onClick={props.stop}>STOP</Button>
      </div>
      <span className="muted">S / M / L: one right turn using saved settings. No item or LLM required.</span>
      <label className="photo-field">Calibration prompt<textarea className="fetch-prompt" rows={10} maxLength={8000} value={draft.prompt}
        disabled={!props.editable || pending} onChange={event => setDraft({ ...draft, prompt: event.target.value })} /></label>
      <Button disabled={!props.editable || pending || JSON.stringify(draft) === JSON.stringify(settings?.values)} onClick={() => void save()}>Save settings</Button>
    </>}
    <span className="muted">Shared with Fetch · original-resolution photos · thinking off · 64 output tokens. Test buttons save edits first.</span>
    <span className="muted">LLM: left/right + large/medium/small. Arm X 10/5/1 mm (capped by setting); base uses the settings above. Both aligned_perfectly = done; otherwise continue. Unknown stops.</span>
    <div className="action-line">
      <Button disabled={disabled} title={props.reasons.join('; ')} onClick={() => void save(true)}>PREVIEW</Button>
      <Button variant="default" disabled={disabled} title={props.reasons.join('; ')} onClick={() => void save(false)}>ALIGN</Button>
      <Button variant="danger" disabled={!props.connected} onClick={props.stop}>STOP</Button>
    </div>
    <span className="muted">Preview: no movement. Align: center the item in the front camera by rotating the base; arm X centers the visible item fragment in the arm camera. Up to 32 corrections, ≤2°/turn and ≤45° total. No translation or gripper motion. Rotation runs first, then fresh photos before arm movement.</span>
    <span className="muted" role="status">{pending ? 'Submitting…' : props.status}</span>
    {props.reasons.length > 0 && <span className="muted">Test needs: {props.reasons.join('; ')}</span>}
    {error && <span role="alert" className="muted">{error}</span>}
    <span className="subheading">LLM ROUNDS</span>
    {!rounds.length && <span className="muted">No requests yet. Latest task only; logs clear when API restarts.</span>}
    {rounds.map(round => {
      const parts = round.messages?.[1].content ?? [];
      return <article className="fetch-round" key={round.id}>
        <header><strong>Round {round.round}</strong><span>{round.mode === 'approach' ? 'Base approach' : round.preview ? 'Preview' : 'Align'} · {round.status}{round.elapsed_s !== undefined && ` · ${round.elapsed_s}s`}</span></header>
        <span className="muted">{new Date(round.started_at_s * 1000).toLocaleTimeString()} · {round.model}</span>
        {round.result_json && <pre>LLM: {round.result_json}</pre>}
        {round.correction && <span>Suggestion: arm X {round.correction.X} mm · base {round.correction.turn_deg}° (positive = clockwise)</span>}
        {round.base_command && <span>Base suggestion: forward {(round.base_command.forward_m * 100).toFixed(0)} cm · turn {round.base_command.turn_deg}°</span>}
        {round.corrected_location && <span>Corrected item: X {round.corrected_location.x_m.toFixed(2)} · Y {round.corrected_location.y_m.toFixed(2)} m · ±{round.corrected_location.uncertainty_m.toFixed(2)} m</span>}
        <span className="muted">Request log; movement completion appears in TASK. History labels show previously executed commands.</span>
        {round.error && <span role="alert" className="muted">{round.error}</span>}
        <div className="fetch-log-images">{parts.map((part, i) => part.image_url && <figure key={i}>
          <img src={part.image_url.url} alt={parts[i - 1]?.text ?? 'LLM camera input'} />
          <figcaption>{parts[i - 1]?.text}</figcaption>
        </figure>)}</div>
        {round.messages && <details><summary>Sent prompt & item</summary><pre>{round.messages[0].content}</pre><pre>{parts[0]?.text}</pre></details>}
      </article>;
    })}
  </section>;
}
