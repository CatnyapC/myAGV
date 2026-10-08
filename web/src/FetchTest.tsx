import { useEffect, useRef, useState } from 'react';
import { api } from './api';
import { Button } from './ui';

type TurnSize = 's' | 'm' | 'l';
type TurnFields = Record<`turn_${TurnSize}_${'speed_rad_s' | 'duration_s'}`, number>;
type FetchSettings = { revision: number; values: TurnFields & { max_step_mm: number; turn_speed_rad_s: number; turn_duration_scale: number; turn_step_deg: number; prompt: string } };
type Part = { type: string; text?: string; image_url?: { url: string } };
type Round = {
  id: string; round: number; mode?: 'grasp' | 'approach'; stage?: 'base' | 'arm'; preview: boolean; status: string; elapsed_s?: number; started_at_s: number;
  model?: string; messages?: [{ content: string }, { content: Part[] }];
  result_json?: string; correction?: { X: number; Y?: number; turn_deg: number }; error?: string;
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
        <label>Arm X/Y step (mm)<input type="number" min={0.1} max={10} step={0.1} value={draft.max_step_mm}
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
    <span className="muted">Shared with Fetch · original-resolution photos · thinking off · 64 output tokens. Test buttons save edits first.</span>
    <span className="muted">Stage 1: front midpoint within 45–55% of image width; base only. After 5 rotations without clear progress, accept coarse position. Stage 2: base locked, arm X/Y match the saved top-camera GOAL, including visible size and cropping. Arm returns visible bounds (0–1000); code computes 10/5/1 mm steps and completion. Unclear bounds or pose stop with an error.</span>
    <div className="action-line">
      <Button disabled={disabled} title={props.reasons.join('; ')} onClick={() => void save(true)}>PREVIEW</Button>
      <Button variant="default" disabled={disabled} title={props.reasons.join('; ')} onClick={() => void save(false)}>ALIGN</Button>
      <Button variant="danger" disabled={!props.connected} onClick={props.stop}>STOP</Button>
    </div>
    <span className="muted">Preview: one coarse-alignment round, no movement. ALIGN runs both stages in order. Arm Z and gripper stay fixed. More bottom fragment than GOAL: Y+; less: Y−. Up to 33 observations; no cumulative arm travel cap.</span>
    <span className="muted" role="status">{pending ? 'Submitting…' : props.status}</span>
    {props.reasons.length > 0 && <span className="muted">Test needs: {props.reasons.join('; ')}</span>}
    {error && <span role="alert" className="muted">{error}</span>}
    <span className="subheading">LLM ROUNDS</span>
    {!rounds.length && <span className="muted">No requests yet. Latest task only; STOP clears logs.</span>}
    {rounds.map(round => {
      const parts = round.messages?.[1].content ?? [];
      return <article className="fetch-round" key={round.id}>
        <header><strong>Round {round.round}</strong><span>{round.mode === 'approach' ? 'Base approach' : round.preview ? 'Preview' : 'Align'} · {round.stage || round.mode} · {round.status}{round.elapsed_s !== undefined && ` · ${round.elapsed_s}s`}</span></header>
        <span className="muted">{new Date(round.started_at_s * 1000).toLocaleTimeString()} · {round.model}</span>
        {round.result_json && <pre>LLM: {round.result_json}</pre>}
        {round.correction && <span>Suggestion: arm X {round.correction.X} mm · Y {round.correction.Y ?? 0} mm · base {round.correction.turn_deg}° (positive = clockwise)</span>}
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
