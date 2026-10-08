import { useEffect, useRef } from 'react';
import L from 'leaflet';
import { chassisOutline, rectangleCorners, toView, toWorld, type ItemLocation, type MapInfo, type Navigation, type Origin, type Photo, type Point, type Zone } from './mapGeometry';

export function MapView(props: {
  info: MapInfo; image: string; zones: Zone[]; selected: string | null;
  drawing: boolean; draft: Point[]; fitVersion: number;
  navigation: Navigation | null; goal: Origin | null; connected: boolean; showCostmap: boolean; focusRobotVersion: number;
  onDraft: (points: Point[]) => void; onSelect: (id: string | null) => void;
  onGoal: (point: Point) => void;
  photos: Photo[]; selectedPhotoId: string | null; onPhoto: (id: string) => void;
  locations: ItemLocation[]; onLocation: (location: ItemLocation) => void;
  update?: { map_id: string; goals: Origin[]; completed: number; active: boolean; state: string };
}) {
  const host = useRef<HTMLDivElement>(null);
  const map = useRef<L.Map | null>(null);
  const latest = useRef(props);
  latest.current = props;
  const preview = useRef<L.Rectangle | null>(null);
  const polygons = useRef<L.LayerGroup | null>(null);
  const bounds = useRef<L.LatLngBounds | null>(null);
  const robot = useRef<L.Polygon | null>(null);
  const heading = useRef<L.Polyline | null>(null);
  const path = useRef<L.Polyline | null>(null);
  const target = useRef<L.CircleMarker | null>(null);
  const targetHeading = useRef<L.Polyline | null>(null);
  const observations = useRef<L.LayerGroup | null>(null);
  const updatePoints = useRef<L.LayerGroup | null>(null);
  const itemPoints = useRef<L.LayerGroup | null>(null);

  useEffect(() => {
    if (!host.current) return;
    const instance = L.map(host.current, { crs: L.CRS.Simple, attributionControl: false,
      minZoom: -7, maxZoom: 7, zoomSnap: 0.25, zoomControl: false });
    L.control.zoom({ position: 'bottomright' }).addTo(instance);
    L.control.scale({ imperial: false, position: 'bottomleft' }).addTo(instance);
    map.current = instance;
    polygons.current = L.layerGroup().addTo(instance);
    observations.current = L.layerGroup().addTo(instance);
    updatePoints.current = L.layerGroup().addTo(instance);
    itemPoints.current = L.layerGroup().addTo(instance);
    path.current = L.polyline([], { color: 'var(--cyan-9)', weight: 2, opacity: .85, interactive: false }).addTo(instance);
    robot.current = L.polygon([], { color: 'var(--cyan-11)', weight: 1.8, fill: false, interactive: false, className: 'robot-outline' }).addTo(instance);
    heading.current = L.polyline([], { color: 'var(--cyan-11)', weight: 1.5, interactive: false }).addTo(instance);
    target.current = L.circleMarker([0, 0], { radius: 5, color: 'var(--orange-9)', weight: 2, fillOpacity: 0, opacity: 0, interactive: false }).addTo(instance);
    targetHeading.current = L.polyline([], { color: 'var(--orange-9)', weight: 1.5, interactive: false }).addTo(instance);
    const observer = new ResizeObserver(() => instance.invalidateSize({ pan: false }));
    observer.observe(host.current);
    instance.on('click', (event: L.LeafletMouseEvent) => {
      const current = latest.current;
      if (!current.drawing) {
        current.onSelect(null);
        current.onGoal(toWorld([event.latlng.lat, event.latlng.lng], current.info.display.origin));
        return;
      }
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
    const nav = props.navigation;
    if (!nav || !nav.pose || nav.map_id !== props.info.map_id) {
      robot.current?.setLatLngs([]); heading.current?.setLatLngs([]); path.current?.setLatLngs([]);
      return;
    }
    const origin = props.info.display.origin;
    robot.current?.setLatLngs(chassisOutline(nav.pose, nav.footprint.length_m, nav.footprint.width_m).map(p => toView(p, origin)));
    robot.current?.getElement()?.classList.toggle('moving', props.connected && nav.moving);
    heading.current?.setLatLngs([toView([nav.pose.x_m, nav.pose.y_m], origin), toView(toWorld([0, nav.footprint.length_m / 2 + .08], nav.pose), origin)]);
    path.current?.setLatLngs(props.connected ? nav.path.map(p => toView(p, origin)) : []);
  }, [props.navigation, props.info, props.connected]);

  useEffect(() => {
    const goal = props.navigation?.moving ? props.navigation.goal : props.goal ?? props.navigation?.goal;
    const color = props.navigation?.moving ? 'var(--orange-9)' : 'var(--cyan-11)';
    target.current?.setStyle({ opacity: goal ? 1 : 0, color });
    targetHeading.current?.setStyle({ color });
    if (goal) {
      target.current?.setLatLng(toView([goal.x_m, goal.y_m], props.info.display.origin));
      targetHeading.current?.setLatLngs([toView([goal.x_m, goal.y_m], props.info.display.origin),
        toView(toWorld([0, .35], goal), props.info.display.origin)]);
    } else targetHeading.current?.setLatLngs([]);
  }, [props.goal, props.navigation, props.info]);

  useEffect(() => {
    const pose = latest.current.navigation?.pose;
    if (pose && props.focusRobotVersion) map.current?.panTo(toView([pose.x_m, pose.y_m], props.info.display.origin), { animate: false });
  }, [props.focusRobotVersion, props.info]);

  useEffect(() => {
    if (!props.showCostmap || !props.navigation || !map.current) return;
    const display = props.info.display;
    const url = `/api/global-costmap.png?view_revision=${display.view_revision}&zone_revision=${props.navigation.costmap.applied_zone_revision}`;
    const image = L.imageOverlay(url, [[0, 0], [display.height_m, display.width_m]],
      { className: 'occupancy-raster', interactive: false, zIndex: 2 }).addTo(map.current);
    return () => { image.remove(); };
  }, [props.showCostmap, props.info, props.navigation?.costmap.applied_zone_revision]);

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
    const group = observations.current;
    if (!group) return;
    group.clearLayers();
    for (const photo of props.photos) {
      if (photo.kind !== 'observation' || photo.map_id !== props.info.map_id || photo.current === false) continue;
      const chosen = photo.id === props.selectedPhotoId;
      const marker = L.circleMarker(toView([photo.base_pose.x_m, photo.base_pose.y_m], props.info.display.origin),
        { radius: chosen ? 6 : 4, color: chosen ? 'var(--cyan-11)' : 'var(--gray-11)', weight: 1.5,
          fillOpacity: .12, dashArray: '2 2', bubblingMouseEvents: false }).addTo(group);
      const label = document.createElement('span'); label.textContent = `Last observed · ${photo.camera_id} · ${new Date(photo.captured_at_s * 1000).toLocaleString()}${photo.source === 'simulation' ? ' · Demo' : ''}`;
      marker.bindTooltip(label).on('click', () => { if (!latest.current.drawing) latest.current.onPhoto(photo.id); });
      const element = marker.getElement();
      element?.setAttribute('tabindex', '0'); element?.setAttribute('role', 'button'); element?.setAttribute('aria-label', label.textContent);
      element?.addEventListener('keydown', event => {
        if ((event as KeyboardEvent).key === 'Enter' || (event as KeyboardEvent).key === ' ') { event.preventDefault(); if (!latest.current.drawing) latest.current.onPhoto(photo.id); }
      });
    }
  }, [props.photos, props.selectedPhotoId, props.info]);

  useEffect(() => {
    const group = itemPoints.current;
    if (!group) return;
    group.clearLayers();
    for (const location of props.locations) {
      if (location.map_id !== props.info.map_id) continue;
      const position = toView([location.x_m, location.y_m], props.info.display.origin);
      L.circle(position, { radius: location.uncertainty_m, color: 'var(--orange-9)', weight: 1,
        dashArray: '3 4', fillOpacity: .04, interactive: false }).addTo(group);
      const icon = L.divIcon({ className: 'item-map-pin', iconSize: [28, 36], iconAnchor: [14, 36],
        tooltipAnchor: [0, -34], html: '<svg viewBox="0 0 24 32" aria-hidden="true"><path d="M12 31S1 19 1 12a11 11 0 0 1 22 0c0 7-11 19-11 19Z"/><circle cx="12" cy="12" r="4"/></svg>' });
      const title = `${location.name} · estimated X ${location.x_m.toFixed(2)}, Y ${location.y_m.toFixed(2)} m`;
      const marker = L.marker(position, { icon, title, alt: title, keyboard: true, bubblingMouseEvents: false }).addTo(group);
      const preview = document.createElement('div'); preview.className = 'item-location-preview';
      const name = document.createElement('strong'); name.textContent = location.name;
      const image = document.createElement('img'); image.src = location.image_url; image.alt = `Observation of ${location.name}`;
      const coordinates = document.createElement('span'); coordinates.textContent = `X ${location.x_m.toFixed(2)} m · Y ${location.y_m.toFixed(2)} m`;
      const detail = document.createElement('small'); detail.textContent = `Estimated ±${location.uncertainty_m.toFixed(2)} m · ${Math.round(location.confidence * 100)}% confidence`;
      const time = document.createElement('small'); time.textContent = new Date(location.captured_at_s * 1000).toLocaleString();
      const action = document.createElement('small'); action.textContent = 'Click to select a reachable approach for Go';
      preview.append(name, image, coordinates, detail, time, action);
      marker.bindTooltip(preview, { direction: 'auto', opacity: 1, className: 'item-location-tooltip' })
        .on('click', () => { if (!latest.current.drawing) latest.current.onLocation(location); });
      marker.getElement()?.addEventListener('focus', () => marker.openTooltip());
      marker.getElement()?.addEventListener('blur', () => marker.closeTooltip());
    }
  }, [props.locations, props.info]);

  useEffect(() => {
    const group = updatePoints.current;
    if (!group) return;
    group.clearLayers();
    const update = props.update;
    if (!update || update.map_id !== props.info.map_id || (!update.active && update.state !== 'planned')) return;
    update.goals.forEach((pose, i) => {
      const point = toView([pose.x_m, pose.y_m], props.info.display.origin);
      const color = i === update.completed ? 'var(--orange-9)' : 'var(--cyan-9)';
      const label = document.createElement('span'); label.textContent = `Update view ${i + 1} · ${(pose.yaw_rad * 180 / Math.PI).toFixed(0)}°`;
      L.circleMarker(point, { radius: 3, color, weight: 1, fillOpacity: .25, interactive: false }).addTo(group).bindTooltip(label);
      L.polyline([point, toView([pose.x_m + .16 * Math.cos(pose.yaw_rad), pose.y_m + .16 * Math.sin(pose.yaw_rad)], props.info.display.origin)],
        { color, weight: 1, interactive: false }).addTo(group);
    });
  }, [props.update, props.info]);

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
