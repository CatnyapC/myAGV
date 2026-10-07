export type Point = [number, number];
export type Origin = { x_m: number; y_m: number; yaw_rad: number };
export type MapInfo = {
  map_id: string; revision: number; frame: string; width: number; height: number;
  resolution_m: number; origin: Origin; png_url: string;
  display: { width_m: number; height_m: number; origin: Origin; view_revision: number; alignment_available: boolean };
};
export type Zone = { id: string; corners: Point[] };
export type Zones = { map_id: string; revision: number; zones: Zone[] };
export type Settings = { revision: number; values: { auto_align: boolean; manual_angle_deg: number } };
type PhotoBase = { id: string; saved_at_s: number; source: string; image_url: string; available: boolean; map_matches: boolean; item_id?: string; association_source?: 'manual' };
export type Photo = PhotoBase & ({ kind: 'reference' } | {
  kind: 'observation'; captured_at_s: number; camera_id: 'front' | 'arm'; map_id: string; map_revision: number; frame: string; base_pose: Origin;
});
export type Item = { id: string; name: string; appearance: string; station_link?: { name: string; map_id: string; digest: string }; station_status: string; fetch_available: false };
export type PhotoIndex = { version: number; revision: number; photos: Photo[]; items: Item[] };
export type Stations = Record<string, { base: { x_m: number; y_m: number; yaw_deg: number }; arm_angles_deg: number[] }>;
export type Navigation = {
  map_id: string; frame: string; source: 'simulation'; stamp_s: number;
  pose: Origin; goal: Origin | null; path: Point[]; moving: boolean; phase: string; status: string;
  footprint: { length_m: number; width_m: number };
  costmap: { ready: boolean; applied_zone_revision: number; clearance_m: number };
};

export function chassisOutline(pose: Origin, length: number, width: number): Point[] {
  const a = length / 2, b = width / 2, cut = Math.min(length, width) * .15;
  return [[-a + cut, -b], [a - cut, -b], [a, -b + cut], [a, b - cut],
    [a - cut, b], [-a + cut, b], [-a, b - cut], [-a, -b + cut]]
    .map(([u, v]) => toWorld([v, u], pose));
}

export function toView([x, y]: Point, origin: Origin): Point {
  const c = Math.cos(origin.yaw_rad), s = Math.sin(origin.yaw_rad);
  const dx = x - origin.x_m, dy = y - origin.y_m;
  return [-s * dx + c * dy, c * dx + s * dy]; // Leaflet [v, u], in meters.
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
