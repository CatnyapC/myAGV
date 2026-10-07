import { useEffect, useRef } from 'react';
import L from 'leaflet';
import { rectangleCorners, toView, type MapInfo, type Point, type Zone } from './mapGeometry';

export function MapView(props: {
  info: MapInfo; image: string; zones: Zone[]; selected: string | null;
  drawing: boolean; draft: Point[]; fitVersion: number;
  onDraft: (points: Point[]) => void; onSelect: (id: string | null) => void;
}) {
  const host = useRef<HTMLDivElement>(null);
  const map = useRef<L.Map | null>(null);
  const latest = useRef(props);
  latest.current = props;
  const preview = useRef<L.Rectangle | null>(null);
  const polygons = useRef<L.LayerGroup | null>(null);
  const bounds = useRef<L.LatLngBounds | null>(null);

  useEffect(() => {
    if (!host.current) return;
    const instance = L.map(host.current, { crs: L.CRS.Simple, attributionControl: false,
      minZoom: -7, maxZoom: 7, zoomSnap: 0.25, zoomControl: false });
    L.control.zoom({ position: 'bottomright' }).addTo(instance);
    L.control.scale({ imperial: false, position: 'bottomleft' }).addTo(instance);
    map.current = instance;
    polygons.current = L.layerGroup().addTo(instance);
    const observer = new ResizeObserver(() => instance.invalidateSize({ pan: false }));
    observer.observe(host.current);
    instance.on('click', (event: L.LeafletMouseEvent) => {
      const current = latest.current;
      if (!current.drawing) { current.onSelect(null); return; }
      const point: Point = [event.latlng.lat, event.latlng.lng];
      current.onDraft(current.draft.length === 1 ? [current.draft[0], point] : [point]);
    });
    instance.on('mousemove', (event: L.LeafletMouseEvent) => {
      const current = latest.current;
      if (current.drawing && current.draft.length === 1) {
        preview.current?.remove();
        preview.current = L.rectangle(L.latLngBounds(current.draft[0], event.latlng),
          { color: 'var(--red-9)', weight: 1, fillOpacity: .18, dashArray: '5 4', interactive: false }).addTo(instance);
      }
    });
    return () => { observer.disconnect(); instance.remove(); map.current = null; };
  }, []);

  useEffect(() => {
    const instance = map.current;
    if (!instance) return;
    const display = props.info.display;
    const nextBounds = L.latLngBounds([0, 0], [display.height_m, display.width_m]);
    const image = L.imageOverlay(props.image, nextBounds, { className: 'occupancy-raster', interactive: false }).addTo(instance);
    bounds.current = nextBounds;
    instance.setMaxBounds(nextBounds.pad(.4));
    instance.fitBounds(nextBounds, { padding: [12, 12], animate: false });
    return () => { image.remove(); };
  }, [props.info, props.image]);

  useEffect(() => {
    if (bounds.current) map.current?.fitBounds(bounds.current, { padding: [12, 12], animate: false });
  }, [props.fitVersion]);

  useEffect(() => {
    const group = polygons.current;
    if (!group) return;
    group.clearLayers();
    for (const zone of props.zones) {
      const chosen = zone.id === props.selected;
      L.polygon(zone.corners.map(p => toView(p, props.info.display.origin)), {
        color: chosen ? 'var(--cyan-11)' : 'var(--red-9)', weight: chosen ? 2 : 1,
        fillColor: 'var(--red-9)', fillOpacity: .24, bubblingMouseEvents: false,
      }).addTo(group).on('click', () => {
        if (!latest.current.drawing) latest.current.onSelect(zone.id);
      });
    }
  }, [props.zones, props.selected, props.info]);

  useEffect(() => {
    preview.current?.remove();
    preview.current = null;
    if (props.draft.length === 2 && map.current) {
      preview.current = L.rectangle(L.latLngBounds(props.draft[0], props.draft[1]),
        { color: 'var(--red-9)', weight: 1, fillOpacity: .18, dashArray: '5 4', interactive: false }).addTo(map.current);
    }
  }, [props.draft]);

  return <div ref={host} className={`map-canvas ${props.drawing ? 'drawing' : ''}`} aria-label="Demo occupancy map" />;
}

export { rectangleCorners };
