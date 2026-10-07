import type { GridStackWidget } from 'gridstack';

export const DEFAULT_LAYOUT: GridStackWidget[] = [
  { id: 'map', x: 0, y: 0, w: 12, h: 12, minW: 1, minH: 1 },
  { id: 'front', x: 12, y: 0, w: 6, h: 6, minW: 1, minH: 1 },
  { id: 'arm', x: 12, y: 6, w: 6, h: 6, minW: 1, minH: 1 },
  { id: 'operate', x: 18, y: 0, w: 6, h: 12, minW: 1, minH: 1 },
];
const MINIMUM: Record<string, [number, number]> = { map: [368, 366], front: [288, 226], arm: [288, 226], operate: [288, 406] };
export type Tile = GridStackWidget & { id: string; x: number; y: number; w: number; h: number };
export type Divider = { axis: 'x' | 'y'; position: number; start: number; end: number; before: string[]; after: string[] };

export function validatedLayout(value: unknown, columns = 24): Tile[] {
  const fallback = () => DEFAULT_LAYOUT.map(item => ({ ...item } as Tile));
  if (!Array.isArray(value) || value.length !== 4) return fallback();
  const ids = new Set();
  const clean: Tile[] = [];
  for (const item of value) {
    const defaults = DEFAULT_LAYOUT.find(w => w.id === item?.id);
    if (!defaults || ids.has(item.id)) return fallback();
    ids.add(item.id);
    const { x, y, w, h } = item;
    if (![x, y, w, h].every(Number.isInteger) || x < 0 || y < 0 || w < 1 || h < 1 || x + w > columns || y > 100 || h > 40) return fallback();
    clean.push({ ...defaults, x, y, w, h } as Tile);
  }
  return clean;
}

export function viewportCellHeight(tiles: Tile[], available: number): number {
  const rows = Math.max(...tiles.map(t => t.y + t.h), 1);
  return Math.max(available / rows, ...tiles.map(t => MINIMUM[t.id][1] / t.h), 1);
}

export function canonicalLayout(canonical: Tile[], previous: Tile[], next: Tile[], columns: number): Tile[] {
  return canonical.map(tile => {
    const before = previous.find(t => t.id === tile.id)!, after = next.find(t => t.id === tile.id)!;
    return { ...tile, x: columns > 1 && after.x !== before.x ? Math.round(after.x * 24 / columns) : tile.x,
      w: columns > 1 && after.w !== before.w ? Math.round(after.w * 24 / columns) : tile.w,
      y: Math.max(0, tile.y + after.y - before.y), h: after.h };
  });
}

export function dividers(tiles: Tile[]): Divider[] {
  const seams: Divider[] = [];
  for (const a of tiles) for (const b of tiles) {
    if (a.id === b.id) continue;
    for (const axis of ['x', 'y'] as const) {
      const size = axis === 'x' ? 'w' : 'h', cross = axis === 'x' ? 'y' : 'x', span = axis === 'x' ? 'h' : 'w';
      const start = Math.max(a[cross], b[cross]), end = Math.min(a[cross] + a[span], b[cross] + b[span]);
      if (a[axis] + a[size] === b[axis] && end > start) {
        seams.push({ axis, position: b[axis], start, end, before: [a.id], after: [b.id] });
      }
    }
  }
  seams.sort((a, b) => a.axis.localeCompare(b.axis) || a.position - b.position || a.start - b.start);
  const merged: Divider[] = [];
  for (const seam of seams) {
    const previous = merged.at(-1);
    if (previous && previous.axis === seam.axis && previous.position === seam.position && seam.start <= previous.end) {
      previous.end = Math.max(previous.end, seam.end);
      previous.before = [...new Set([...previous.before, ...seam.before])].sort();
      previous.after = [...new Set([...previous.after, ...seam.after])].sort();
    } else merged.push({ ...seam });
  }
  return merged;
}

export function moveDivider(tiles: Tile[], seam: Divider, requested: number, cellWidth: number, cellHeight: number): Tile[] {
  const size = seam.axis === 'x' ? 'w' : 'h';
  const unit = seam.axis === 'x' ? cellWidth : cellHeight;
  const minimum = (tile: Tile) => Math.max(1, Math.ceil(MINIMUM[tile.id][seam.axis === 'x' ? 0 : 1] / unit));
  const before = tiles.filter(t => seam.before.includes(t.id)), after = tiles.filter(t => seam.after.includes(t.id));
  const low = Math.max(...before.map(t => minimum(t) - t[size]));
  const high = Math.min(...after.map(t => t[size] - minimum(t)));
  if (low > high) return tiles;
  const delta = Math.max(low, Math.min(high, Math.round(requested)));
  return tiles.map(t => seam.before.includes(t.id) ? { ...t, [size]: t[size] + delta } :
    seam.after.includes(t.id) ? { ...t, [seam.axis]: t[seam.axis] + delta, [size]: t[size] - delta } : { ...t });
}
