export type LogLevel = 'info' | 'error';
export type LogReporter = (message: string, level?: LogLevel, source?: string) => void;
export type LogEntry = { id: number; time: number; message: string; level: LogLevel; source: string; repeats: number };

export function appendLog(entries: LogEntry[], message: string, level: LogLevel = 'info', source = 'System', time = Date.now()): LogEntry[] {
  if (!message.trim()) return entries;
  const last = entries.at(-1);
  if (last?.message === message && last.level === level && last.source === source && time - last.time < 5000) {
    return [...entries.slice(0, -1), { ...last, time, repeats: last.repeats + 1 }];
  }
  // shortcut: keep this tab's latest 200 messages; use server storage for persistent history.
  return [...entries.slice(-199), { id: (last?.id ?? 0) + 1, time, message, level, source, repeats: 1 }];
}
