import { useEffect, useRef, useState } from 'react';
import { api } from './api';
import { Button } from './ui';
import { MeasuredPhoto } from './MeasuredPhoto';

type TurnSize = 's' | 'm' | 'l';
type TurnFields = Record<`turn_${TurnSize}_${'speed_rad_s' | 'duration_s'}`, number>;
type FetchSettings = { revision: number; values: TurnFields & { max_step_mm: number; turn_speed_rad_s: number; turn_duration_scale: number; turn_step_deg: number; prompt: string } };
type Part = { type: string; text?: string; image_url?: { url: string } };
type Round = {
  id: string; round: number; mode?: 'grasp' | 'approach'; stage?: 'base' | 'arm'; preview: boolean; status: string; elapsed_s?: number; started_at_s: number;
  model?: string; messages?: [{ content: string }, { content: Part[] }];
  result_json?: string; correction?: { X: number; Y?: number; turn_deg: number }; error?: string;
  measurement?: { goal: number[]; current: number[] };
  residual_mm?: { x_mm: number; y_mm: number }; metric_ready?: boolean;
  usage?: { inputTokens?: number; cachedInputTokens?: number }; codex?: { thread_id: string; turn_id: string; service_tier?: string };
  measured_images?: { label: string; image: string; bounds?: number[] }[];
  base_command?: { forward_m: number; turn_deg: number }; corrected_location?: { x_m: number; y_m: number; uncertainty_m: number };
};

export function FetchTest(props: {
  connected: boolean; editable: boolean; turnEnabled: boolean; reasons: string[]; itemId: string; indexRevision?: number;
  itemName?: string; graspZ?: number; metric?: boolean; status?: string; command: (type: string, values: Record<string, unknown>) => Promise<void>; stop: () => void;
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
  async function graspTest() {
    if (pending) return;
    setPending(true); setError('');
    try {
      await props.command('grasp_test', { item_id: props.itemId, index_revision: props.indexRevision });
    } catch (failure) { setError((failure as Error).message); }
    finally { setPending(false); }
  }
  const disabled = pending || !settings || !draft || props.reasons.length > 0;
  return <section className="fetch-test" aria-label="Grasp calibration test">
    <div className="section-divider" /><h3 className="subheading">GRASP TEST</h3>
    <span className="muted">Item: {props.itemName || 'Select an item above'} · Align accepts J1 75–105° near the object. No map-pose confirmation needed for this local test.</span>
    {draft && <>
      <div className="goal-inputs">
        <label>Arm X/Y step (mm)<input type="number" min={0.1} max={20} step={0.1} value={draft.max_step_mm}
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
    <span className="muted">Shared with Fetch · original-resolution photos. Alignment and rotation tests save edits first.</span>
    <span className="muted">{props.metric ? 'Stage 1: base coarse alignment. Stage 2: two measured X/Y probes (up to 5 mm each) establish millimeter scale automatically. Model alignment: planar error <1 mm. Controller alignment: full |X| + |Y| <1.25 mm before step limits. Larger corrections use up to the saved 20 mm limit. Missing or ambiguous targets stop.' : 'Stage 1: base coarse alignment. Stage 2: arm X/Y match GOAL using visible bounds: top within 0.5%, horizontal/bottom within 2%, width within 10%, area within 15%. Missing or ambiguous targets stop.'}</span>
    <div className="action-line">
      <Button disabled={disabled} title={props.reasons.join('; ')} onClick={() => void save(true)}>PREVIEW</Button>
      <Button variant="default" disabled={disabled} title={props.reasons.join('; ')} onClick={() => void save(false)}>ALIGN</Button>
      <Button disabled={disabled || props.graspZ == null}
        title={props.reasons.join('; ') || (props.graspZ == null ? 'Save the item grasp height first.' : 'Run after ALIGN. Pick up and put down at the current X/Y; base stays stopped.')}
        onClick={() => void graspTest()}>GRASP / PLACE</Button>
      <Button variant="danger" disabled={!props.connected} onClick={props.stop}>STOP</Button>
    </div>
    <span className="muted">Preview: one coarse-alignment round, no movement. ALIGN first restores ARM GOAL Z within 1 mm, then runs both stages with Z fixed. Gripper stays fixed. Target above GOAL: Y−; below: Y+. Up to 33 observations; no cumulative arm travel cap.</span>
    <span className="muted">Grasp / place: {props.graspZ == null ? 'record the item grasp Z first' : `saved grasp Z ${props.graspZ.toFixed(1)} mm`}. After ALIGN, open, lower to grasp Z, close, lift to starting Z, lower and release, then return to starting Z. X/Y stay fixed.</span>
    <span className="muted" role="status">{pending ? 'Submitting…' : props.status}</span>
    {props.reasons.length > 0 && <span className="muted">Test needs: {props.reasons.join('; ')}</span>}
    {error && <span role="alert" className="muted">{error}</span>}
    <span className="subheading">ALIGNMENT ROUNDS</span>
    {!rounds.length && <span className="muted">No requests yet. Latest task only; STOP clears logs.</span>}
    {rounds.map(round => {
      const parts = round.messages?.[1].content ?? [];
      return <article className="fetch-round" key={round.id}>
        <header><strong>Round {round.round}</strong><span>{round.mode === 'approach' ? 'Base approach' : round.preview ? 'Preview' : 'Align'} · {round.stage || round.mode} · {round.status}{round.elapsed_s !== undefined && ` · ${round.elapsed_s}s`}</span></header>
        <span className="muted">{new Date(round.started_at_s * 1000).toLocaleTimeString()} · {round.model}</span>
        {round.result_json && <pre>LLM: {round.result_json}</pre>}
        {round.measurement && <pre>Measured: {JSON.stringify(round.measurement)}</pre>}
        {round.metric_ready === false && <span className="muted">Measuring millimeter scale; alignment cannot finish yet.</span>}
        {round.residual_mm && <span>Remaining error: X {round.residual_mm.x_mm.toFixed(2)} mm · Y {round.residual_mm.y_mm.toFixed(2)} mm · total {(Math.abs(round.residual_mm.x_mm) + Math.abs(round.residual_mm.y_mm)).toFixed(2)} mm</span>}
        {round.codex && <span className="muted">Codex {round.codex.service_tier} · thread {round.codex.thread_id.slice(-8)}{round.usage?.inputTokens ? ` · cache ${Math.round(100 * (round.usage.cachedInputTokens ?? 0) / round.usage.inputTokens)}%` : ''}</span>}
        {round.correction && <span>Suggestion: arm X {round.correction.X} mm · Y {round.correction.Y ?? 0} mm · base {round.correction.turn_deg}° (positive = clockwise)</span>}
        {round.base_command && <span>Base suggestion: forward {(round.base_command.forward_m * 100).toFixed(0)} cm · turn {round.base_command.turn_deg}°</span>}
        {round.corrected_location && <span>Corrected item: X {round.corrected_location.x_m.toFixed(2)} · Y {round.corrected_location.y_m.toFixed(2)} m · ±{round.corrected_location.uncertainty_m.toFixed(2)} m</span>}
        <span className="muted">Request log; movement completion appears in TASK. History labels show previously executed commands.</span>
        {round.error && <span role="alert" className="muted">{round.error}</span>}
        <div className="fetch-log-images">{round.measured_images?.map(view => <figure key={view.label}>
          <MeasuredPhoto src={view.image} alt={view.label} bounds={view.bounds} /><figcaption>{view.label}{view.bounds && ` · ${view.bounds.join(', ')}`}</figcaption>
        </figure>)}{parts.map((part, i) => part.image_url && <figure key={i}>
          <img src={part.image_url.url} alt={parts[i - 1]?.text ?? 'LLM camera input'} />
          <figcaption>{parts[i - 1]?.text}</figcaption>
        </figure>)}</div>
        {round.messages && <details><summary>Sent prompt & item</summary><pre>{round.messages[0].content}</pre><pre>{parts[0]?.text}</pre></details>}
      </article>;
    })}
  </section>;
}
