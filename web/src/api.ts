export async function api<T>(url: string, options: RequestInit = {}): Promise<T> {
  const response = await fetch(url, { ...options, headers: { 'Content-Type': 'application/json', ...options.headers } });
  const body = await response.json().catch(() => ({ error: `Request failed (${response.status})` }));
  if (!response.ok) throw new Error(body.error ?? `Request failed (${response.status})`);
  return body as T;
}

// getRandomValues also works on the supervised LAN HTTP origin.
export function commandId(): string {
  const b = crypto.getRandomValues(new Uint8Array(16));
  b[6] = (b[6] & 15) | 64; b[8] = (b[8] & 63) | 128;
  const hex = Array.from(b, n => n.toString(16).padStart(2, '0')).join('');
  return `${hex.slice(0,8)}-${hex.slice(8,12)}-${hex.slice(12,16)}-${hex.slice(16,20)}-${hex.slice(20)}`;
}

export type HardwareState = {
  stop_epoch: number; phase: string; status: string; task_id?: string; arm_available: boolean; arm_homed: boolean;
  arm_angles?: number[]; arm_error?: string; navigation_ready: boolean; motion_available: boolean;
  localized: boolean; driver_watchdog: boolean; exclusive: boolean; sensors_ready: boolean;
  transport_angles?: number[]; clearance_m: number; zones_ready: boolean; planner_footprint_ready: boolean; stop_confirmed: boolean;
};

export function hardwareBlocks(state?: HardwareState): string[] {
  if (!state) return ['Controller unavailable'];
  const reasons: string[] = [];
  if (state.phase === 'fault') reasons.push('Verify stopped hardware and clear the fault');
  else if (state.phase !== 'idle') reasons.push('Finish or stop the current task');
  if (!state.driver_watchdog) reasons.push('Driver watchdog unavailable');
  if (!state.sensors_ready) reasons.push('Waiting for fresh LiDAR and odometry');
  if (!state.exclusive) reasons.push('Close competing chassis controllers');
  if (!state.localized) reasons.push('Confirm the robot position and heading on the map');
  if (!state.arm_available) reasons.push('Connect P340');
  else if (!state.arm_homed) reasons.push('Home arm, or confirm already homed');
  if (!state.transport_angles) reasons.push('Record folded transport pose and measured clearance');
  if (!state.zones_ready) reasons.push('Waiting for both ROS costmaps');
  if (!state.planner_footprint_ready) reasons.push('Waiting for the measured planner footprint');
  if (!state.navigation_ready && !reasons.length) reasons.push('Controller not ready');
  return reasons;
}
