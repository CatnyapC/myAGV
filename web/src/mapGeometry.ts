export type Point = [number, number];
export type Origin = { x_m: number; y_m: number; yaw_rad: number };
export type ObservationPoint = Origin & { id: string };
export type ObservationPoints = { map_id: string; revision: number; points: ObservationPoint[] };
export type MapInfo = {
  map_id: string; revision: number; frame: string; width: number; height: number;
  resolution_m: number; origin: Origin; png_url: string;
  display: { width_m: number; height_m: number; origin: Origin; view_revision: number; alignment_available: boolean; content_bounds?: [Point, Point] };
};
export type Zone = { id: string; corners: Point[] };
export type Zones = { map_id: string; revision: number; zones: Zone[] };
export type Settings = { revision: number; values: { auto_align: boolean; manual_angle_deg: number; llm_model: string; reasoning_effort: string; fast_mode: boolean };
  llm_models: { id: string; name: string; efforts: string[] }[] };
type PhotoBase = { id: string; saved_at_s: number; source: string; image_url: string; available: boolean; map_matches: boolean; item_id?: string; association_source?: 'manual'; current?: boolean; update_key?: string;
  location_analysis?: { status: 'queued' | 'running' | 'complete' | 'error' | 'cancelled'; error?: string } };
export type Photo = PhotoBase & ({ kind: 'reference' } | {
  kind: 'observation'; captured_at_s: number; camera_id: 'front' | 'arm'; map_id: string; map_revision: number; frame: string; base_pose: Origin; arm_angles_deg?: number[];
});
export type Item = { id: string; name: string; appearance: string; station_link?: { name: string; map_id: string; digest: string }; station_status: string; fetch_available: boolean; observation_current: boolean; last_seen_s: number;
  reference_photo_id?: string; grasp_z_mm?: number; calibration_z_mm?: number; grasp_goal?: { front: string; arm: string }; grasp_goal_ready: boolean };
export type ItemLocation = { item_id: string; photo_id: string; name: string; image_url: string; map_id: string;
  x_m: number; y_m: number; uncertainty_m: number; confidence: number; captured_at_s: number; base_pose: Origin };
export type PhotoIndex = { version: number; revision: number; photos: Photo[]; items: Item[]; locations?: ItemLocation[] };
export type Stations = Record<string, { base: { x_m: number; y_m: number; yaw_deg: number }; arm_angles_deg: number[] }>;

export function latestMapObservation(photos: Photo[], mapId: string) {
  let latest: Extract<Photo, { kind: 'observation' }> | undefined;
  for (const photo of photos) {
    if (photo.kind !== 'observation' || photo.map_id !== mapId || photo.current === false) continue;
    if (!latest || photo.captured_at_s >= latest.captured_at_s) latest = photo;
  }
  return latest;
}
export type Navigation = {
  map_id: string; frame: string; source: 'simulation' | 'ros'; stamp_s: number;
  pose: Origin | null; goal: Origin | null; path: Point[]; moving: boolean; phase: string; status: string;
  footprint: { length_m: number; width_m: number };
  costmap: { ready: boolean; applied_zone_revision: number; clearance_m: number };
};

export function chassisOutline(pose: Origin, length: number, width: number): Point[] {
  const a = length / 2, b = width / 2, nose = length * .25;
  return [[-a, -b], [a - nose, -b], [a, 0], [a - nose, b], [-a, b]]
    .map(([u, v]) => toWorld([v, u], pose));
}

export function toView([x, y]: Point, origin: Origin): Point {
  const c = Math.cos(origin.yaw_rad), s = Math.sin(origin.yaw_rad);
  const dx = x - origin.x_m, dy = y - origin.y_m;
  return [-s * dx + c * dy, c * dx + s * dy]; // Leaflet [v, u], in meters.
}
export function mapFitBounds(info: MapInfo, navigation: Navigation | null): [Point, Point] {
  const display = info.display;
  const bounds: [Point, Point] = display.content_bounds ?? [[0, 0], [display.height_m, display.width_m]];
  const pose = navigation?.map_id === info.map_id ? navigation.pose : null;
  if (!pose || !navigation) return bounds;
  const [y, x] = toView([pose.x_m, pose.y_m], display.origin);
  const points = [...bounds, ...chassisOutline(pose, navigation.footprint.length_m, navigation.footprint.width_m)
    .map(point => toView(point, display.origin))];
  // Keep the robot centered while leaving room for all explored cells.
  const halfHeight = Math.max(...points.map(point => Math.abs(point[0] - y)));
  const halfWidth = Math.max(...points.map(point => Math.abs(point[1] - x)));
  return [[y - halfHeight, x - halfWidth], [y + halfHeight, x + halfWidth]];
}
export function toWorld([v, u]: Point, origin: Origin): Point {
  const c = Math.cos(origin.yaw_rad), s = Math.sin(origin.yaw_rad);
  return [origin.x_m + c * u - s * v, origin.y_m + s * u + c * v];
}
export function rectangleCorners(a: Point, b: Point, origin: Origin): Point[] {
  const loV = Math.min(a[0], b[0]), hiV = Math.max(a[0], b[0]);
  const loU = Math.min(a[1], b[1]), hiU = Math.max(a[1], b[1]);
  return [[loV, loU], [loV, hiU], [hiV, hiU], [hiV, loU]].map(p => toWorld(p as Point, origin));
}
