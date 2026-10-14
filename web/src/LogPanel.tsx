import { useState } from 'react';
import { ChevronDown, ChevronUp, Trash2 } from 'lucide-react';
import { Button } from './ui';
import type { LogEntry } from './activityLog';

export function LogPanel({ entries, onClear }: { entries: LogEntry[]; onClear: () => void }) {
  const [expanded, setExpanded] = useState(false);
  return <section className={`log-panel ${expanded ? 'expanded' : ''}`} aria-label="Activity log">
    <header className="log-header"><h2>LOG</h2><div className="panel-tools">
      <Button variant="ghost" className="icon" aria-label="Clear log" title="Clear log" disabled={!entries.length} onClick={onClear}><Trash2 size={13} /></Button>
      <Button variant="ghost" className="icon" aria-label={expanded ? 'Collapse log' : 'Expand log'} title={expanded ? 'Collapse log' : 'Expand log'}
        aria-expanded={expanded} aria-controls="activity-log" onClick={() => setExpanded(value => !value)}>{expanded ? <ChevronUp size={15} /> : <ChevronDown size={15} />}</Button>
    </div></header>
    <div id="activity-log" className="log-entries" role="log" aria-label="Activity messages" aria-live="polite" tabIndex={0}>
      {entries.length ? [...entries].reverse().map(entry => <div key={entry.id} className={`log-entry ${entry.level}`}>
        <time dateTime={new Date(entry.time).toISOString()}>{new Date(entry.time).toLocaleTimeString([], { hour12: false })}</time>
        <span className="log-source">{entry.source}</span><span className="log-message">{entry.level === 'error' && <span className="sr-only">Error: </span>}{entry.message}{entry.repeats > 1 && <small> ×{entry.repeats}</small>}</span>
      </div>) : <span className="log-empty">No activity yet</span>}
    </div>
  </section>;
}
