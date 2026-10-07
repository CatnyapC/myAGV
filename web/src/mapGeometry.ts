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
