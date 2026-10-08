import { useEffect, useState } from 'react';
import { Battery } from 'lucide-react';
import { api } from './api';

export function BatteryStatus() {
  const [voltage, setVoltage] = useState<number | null>(null);
  useEffect(() => {
    const controller = new AbortController();
    const refresh = async () => {
      try {
        const battery = await api<{ voltage_v: number } | null>('/api/battery', { signal: controller.signal });
        if (!controller.signal.aborted) setVoltage(battery?.voltage_v ?? null);
      } catch {
        if (!controller.signal.aborted) setVoltage(null);
      }
    };
    void refresh();
    const timer = setInterval(() => void refresh(), 60_000);
    return () => { clearInterval(timer); controller.abort(); };
  }, []);
  return <output className="battery-status" aria-label="Battery voltage" title="Highest battery pack voltage · refreshes every 60 seconds">
    <Battery size={16} aria-hidden="true" /><span>{voltage === null ? 'Unavailable' : `${voltage.toFixed(1)} V`}</span>
  </output>;
}
