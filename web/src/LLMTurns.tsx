import { useEffect, useRef, useState } from 'react';
import { api } from './api';
import { MeasuredPhoto } from './MeasuredPhoto';
import type { LogReporter } from './activityLog';

type Part = { type: string; text?: string; image_url?: { url: string } };
type Round = {
  id: string; round: number; mode?: 'grasp' | 'approach' | 'locations'; stage?: 'base' | 'arm'; preview: boolean; status: string; elapsed_s?: number; started_at_s: number;
  model?: string; messages?: [{ content: string }, { content: Part[] }];
  result_json?: string; correction?: { X: number; Y?: number; turn_deg: number }; error?: string;
  measurement?: { goal: number[]; current: number[] };
  residual_mm?: { x_mm: number; y_mm: number }; metric_ready?: boolean; calibration_reused?: boolean;
  usage?: { inputTokens?: number; cachedInputTokens?: number }; codex?: { thread_id: string; turn_id: string; service_tier?: string };
  measured_images?: { label: string; image: string; bounds?: number[] }[];
  base_command?: { forward_m: number; turn_deg: number }; corrected_location?: { x_m: number; y_m: number; uncertainty_m: number };
  outcome?: 'arrived' | 'stand_off';
};

export function LLMTurns(props: { connected: boolean; onLog: LogReporter }) {
  const [rounds, setRounds] = useState<Round[]>([]);
  const roundStatus = useRef(new Map<string, string>());
  const logRevision = useRef(-1);
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
      } catch (failure) { if (!controller.signal.aborted) props.onLog((failure as Error).message, 'error', 'Calibration'); }
      if (!controller.signal.aborted) timer = setTimeout(() => void poll(), 1000);
    }
    // Reconnect may reach a restarted API with the same revision but no prior logs.
    logRevision.current = -1;
    void poll();
    return () => { controller.abort(); clearTimeout(timer); };
  }, [props.connected, props.onLog]);
  useEffect(() => {
    const next = new Map<string, string>();
    for (const round of rounds) {
      const status = `${round.status}:${round.error ?? ''}`;
      next.set(round.id, status);
      if (roundStatus.current.get(round.id) !== status) {
        props.onLog(`Round ${round.round} · ${round.mode === 'locations' ? 'Item positions' : round.preview ? 'Preview' : round.mode === 'approach' ? 'Approach' : 'Align'} · ${round.status}${round.error ? ` · ${round.error}` : ''}`, round.error ? 'error' : 'info', 'Calibration');
      }
    }
    roundStatus.current = next;
  }, [rounds, props.onLog]);
  return <div className="llm-turns">
    <span className="muted" role="status">{rounds.length ? `${rounds.length} turns` : 'No LLM turns yet. Update item positions, or run PREVIEW, ALIGN or FETCH.'}</span>
    {rounds.map(round => {
      const parts = round.messages?.[1].content ?? [];
      return <article className="fetch-round" key={round.id}>
        <header><strong>Round {round.round}</strong><span>{round.mode === 'locations' ? 'Item positions' : round.mode === 'approach' ? 'Base approach' : round.preview ? 'Preview' : 'Align'} · {round.stage || round.mode} · {round.status}{round.elapsed_s !== undefined && ` · ${round.elapsed_s}s`}</span></header>
        <span className="muted">{new Date(round.started_at_s * 1000).toLocaleTimeString()} · {round.model}</span>
        {round.result_json && <pre>Output: {round.result_json}</pre>}
        <div className="fetch-log-images">{round.measured_images?.map(view => <figure key={view.label}>
          <MeasuredPhoto src={view.image} alt={view.label} bounds={view.bounds} /><figcaption>{view.label}{view.bounds && ` · ${view.bounds.join(', ')}`}</figcaption>
        </figure>)}{parts.map((part, i) => part.image_url && <figure key={i}>
          <img src={part.image_url.url} alt={parts[i - 1]?.text ?? 'LLM camera input'} />
          <figcaption>{parts[i - 1]?.text}</figcaption>
        </figure>)}</div>
        {round.measurement && <pre>Measured: {JSON.stringify(round.measurement)}</pre>}
        {round.metric_ready === false && <span className="muted">Measuring millimeter scale; alignment cannot finish yet.</span>}
        {round.calibration_reused && <span className="muted">Reusing this GOAL's measured millimeter scale; no repeated probes.</span>}
        {round.residual_mm && <span>Remaining error: X {round.residual_mm.x_mm.toFixed(2)} mm · Y {round.residual_mm.y_mm.toFixed(2)} mm · total {(Math.abs(round.residual_mm.x_mm) + Math.abs(round.residual_mm.y_mm)).toFixed(2)} mm</span>}
        {round.codex && <span className="muted">Codex {round.codex.service_tier} · thread {round.codex.thread_id.slice(-8)}{round.usage?.inputTokens ? ` · cache ${Math.round(100 * (round.usage.cachedInputTokens ?? 0) / round.usage.inputTokens)}%` : ''}</span>}
        {round.correction && <span>Suggestion: arm X {round.correction.X} mm · Y {round.correction.Y ?? 0} mm · base {round.correction.turn_deg}° (positive = clockwise)</span>}
        {round.base_command && <span>Base suggestion: {round.base_command.forward_m < 0 ? 'backward' : 'forward'} {Math.abs(round.base_command.forward_m * 100).toFixed(0)} cm · turn {round.base_command.turn_deg}°</span>}
        {round.corrected_location && <span>Corrected item: X {round.corrected_location.x_m.toFixed(2)} · Y {round.corrected_location.y_m.toFixed(2)} m · ±{round.corrected_location.uncertainty_m.toFixed(2)} m</span>}
        {round.outcome === 'arrived' && <span>Saved final GOAL front view matched; base approach complete.</span>}
        <span className="muted">Request history · movement completion appears in LOG.</span>
        {round.error && <span role="alert" className="muted">{round.error}</span>}
        {round.messages && <details><summary>Prompt</summary><pre>{round.messages[0].content}</pre>
          {parts.map((part, i) => part.text && <pre key={i}>{part.text}</pre>)}
        </details>}
      </article>;
    })}
  </div>;
}
