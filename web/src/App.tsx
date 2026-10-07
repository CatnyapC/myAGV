import { useCallback, useEffect, useRef, useState, type ReactNode, type PointerEvent } from 'react';
import * as Dialog from '@radix-ui/react-dialog';
import * as Dropdown from '@radix-ui/react-dropdown-menu';
import { GridStack } from 'gridstack';
import { Camera, Crosshair, Expand, LayoutGrid, Menu, MoreHorizontal, RotateCcw, Settings2, Shrink, Square, Trash2, X } from 'lucide-react';
import { MapView, rectangleCorners } from './MapView';
import { type MapInfo, type Navigation, type Point, type Settings, type Zones } from './mapGeometry';
import { Button } from './ui';
import { DEFAULT_LAYOUT, canonicalLayout, dividers, moveDivider, validatedLayout, viewportCellHeight, type Divider, type Tile } from './layout';

type State = { robot_id: string; demo: boolean; phase: string; operator_active: boolean; navigation: Navigation };
const LAYOUT_KEY = 'myagv:local-demo:layout:v1';
const names: Record<string, string> = { map: 'MAP', front: 'FRONT CAMERA', arm: 'ARM CAMERA', operate: 'OPERATE' };

async function api<T>(url: string, options: RequestInit = {}): Promise<T> {
  const response = await fetch(url, { ...options, headers: { 'Content-Type': 'application/json', ...options.headers } });
  const body = await response.json().catch(() => ({ error: `Request failed (${response.status})` }));
  if (!response.ok) throw new Error(body.error ?? `Request failed (${response.status})`);
  return body as T;
}

export function App() {
  const [state, setState] = useState<State | null>(null);
  const [connected, setConnected] = useState(false);
  const [packet, setPacket] = useState<{ info: MapInfo; image: string } | null>(null);
  const [zones, setZones] = useState<Zones | null>(null);
  const [savedSettings, setSavedSettings] = useState<Settings | null>(null);
  const [settingsDraft, setSettingsDraft] = useState({ auto_align: true, manual_angle_deg: 0 });
  const [angleText, setAngleText] = useState('0');
  const [settingsSaving, setSettingsSaving] = useState(false);
  const [saveError, setSaveError] = useState('');
  const [lease, setLease] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState('');
  const [drawing, setDrawing] = useState(false);
  const [draft, setDraft] = useState<Point[]>([]);
  const [selected, setSelected] = useState<string | null>(null);
  const [fitVersion, setFitVersion] = useState(0);
  const [focusRobotVersion, setFocusRobotVersion] = useState(0);
  const [showCostmap, setShowCostmap] = useState(false);
  const [goalText, setGoalText] = useState(['', '', '0']);
  const [settingsOpen, setSettingsOpen] = useState(false);
  const [layoutEditing, setLayoutEditing] = useState(false);
  const [layoutError, setLayoutError] = useState('');
  const [maximized, setMaximized] = useState<string | null>(null);
  const [layout, setLayout] = useState<Tile[]>(validatedLayout(DEFAULT_LAYOUT));
  const [gridSize, setGridSize] = useState({ columns: 24, cellHeight: 40 });
  const [keyboardCorners, setKeyboardCorners] = useState(['1', '1', '3', '3']);
  const gridHost = useRef<HTMLDivElement>(null);
  const workspaceHost = useRef<HTMLDivElement>(null);
  const grid = useRef<GridStack | null>(null);
  const fittingLayout = useRef(false);
  const fitViewport = useRef<() => void>(() => {});
  const draggingDivider = useRef<{ seam: Divider; tiles: Tile[]; start: number; cellWidth: number; cellHeight: number } | null>(null);
  const generation = useRef(0);
  const previousMap = useRef('');
  const eventSocket = useRef<WebSocket | null>(null);
  const operatorLease = useRef(lease);
  operatorLease.current = lease;
  const editingSettings = useRef(false);
  const settingsDirty = savedSettings !== null && JSON.stringify(settingsDraft) !== JSON.stringify(savedSettings.values);
  const angleValid = angleText.trim() !== '' && Number.isFinite(Number(angleText)) && Math.abs(Number(angleText)) <= 180;
  editingSettings.current = settingsDirty || settingsSaving || !angleValid;
  const canEdit = Boolean(lease && connected && state?.phase === 'idle');
  const canLayout = connected && state?.phase === 'idle';
  const canResize = canLayout && layoutEditing && !maximized;
  const navigation = state?.navigation ?? null;
  const goalValid = goalText.every(v => v.trim() !== '' && Number.isFinite(Number(v))) && Math.abs(Number(goalText[2])) <= 180;
  const goal = goalValid ? { x_m: Number(goalText[0]), y_m: Number(goalText[1]), yaw_rad: Number(goalText[2]) * Math.PI / 180 } : null;
  const costmapReady = navigation?.costmap.ready && navigation.costmap.applied_zone_revision === zones?.revision;

  const reload = useCallback(async () => {
    const current = ++generation.current;
    const [nextState, info, nextZones, preferences] = await Promise.all([
      api<State>('/api/state'), api<MapInfo>('/api/map'), api<Zones>('/api/no-go-zones'), api<Settings>('/api/settings'),
    ]);
    const imageResponse = await fetch(info.png_url);
    if (!imageResponse.ok) throw new Error('Map changed; reload required');
    const image = URL.createObjectURL(await imageResponse.blob());
    try {
      const decoded = new Image(); decoded.src = image; await decoded.decode();
      if (current !== generation.current) { URL.revokeObjectURL(image); return; }
      const key = `${info.map_id}:${info.display.view_revision}`;
      if (key !== previousMap.current) { setDraft([]); setDrawing(false); }
      if (!previousMap.current.startsWith(`${info.map_id}:`)) { setSelected(null); setGoalText(['', '', '0']); }
      previousMap.current = key;
      setPacket(old => {
        // Keep map/pan intact on zone or lease events; replace raster only on a new view.
        if (old && old.info.map_id === info.map_id && old.info.revision === info.revision && old.info.display.view_revision === info.display.view_revision) {
          URL.revokeObjectURL(image); return old;
        }
        return { info, image };
      });
      setState(old => old && old.navigation.stamp_s > nextState.navigation.stamp_s ? old : nextState);
      setZones(nextZones); setSavedSettings(preferences);
      if (!editingSettings.current) { setSettingsDraft(preferences.values); setAngleText(String(preferences.values.manual_angle_deg)); }
    } catch (failure) { URL.revokeObjectURL(image); throw failure; }
  }, []);

  useEffect(() => () => { if (packet) URL.revokeObjectURL(packet.image); }, [packet]);

  useEffect(() => {
    let disposed = false, timer: ReturnType<typeof setTimeout>, socket: WebSocket | null = null;
    const connect = async () => {
      try {
        await reload();
        if (disposed) return;
        socket = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/api/events`);
        eventSocket.current = socket;
        socket.onopen = () => { setConnected(true); if (operatorLease.current) socket?.send(JSON.stringify({ type: 'operator', lease: operatorLease.current })); };
        socket.onmessage = event => {
          const message = JSON.parse(event.data) as { type: string; state?: State };
          if (message.type === 'telemetry' && message.state) {
            setState(message.state);
            if (!message.state.operator_active) setLease(null);
          } else void reload().catch(failure => { setError(failure.message); });
        };
        socket.onclose = () => {
          if (!disposed) { setConnected(false); setLease(null); setDrawing(false); setDraft([]); timer = setTimeout(connect, 1500); }
        };
        socket.onerror = () => socket?.close();
      } catch (failure) {
        if (!disposed) { setConnected(false); setError((failure as Error).message); timer = setTimeout(connect, 1500); }
      }
    };
    void connect();
    return () => { disposed = true; clearTimeout(timer); socket?.close(); eventSocket.current = null; ++generation.current; };
  }, [reload]);

  useEffect(() => {
    if (lease && eventSocket.current?.readyState === WebSocket.OPEN) eventSocket.current.send(JSON.stringify({ type: 'operator', lease }));
  }, [lease, connected]);

  useEffect(() => {
    if (!lease || !connected) return;
    const timer = setInterval(() => {
      void api('/api/control/renew', { method: 'POST', headers: { 'X-Control-Lease': lease } }).catch(() => {
        setLease(null); setDrawing(false); setDraft([]); setError('Editing lease expired');
      });
    }, 10000);
    return () => clearInterval(timer);
  }, [lease, connected]);

  useEffect(() => {
    if (!canEdit) { setDrawing(false); setDraft([]); }
  }, [canEdit]);
  useEffect(() => { if (!canLayout) setLayoutEditing(false); if (!canResize) draggingDivider.current = null; }, [canLayout, canResize]);
  useEffect(() => {
    const key = (event: KeyboardEvent) => { if (event.key === 'Escape') { setDrawing(false); setDraft([]); setMaximized(null); } };
    window.addEventListener('keydown', key);
    return () => window.removeEventListener('keydown', key);
  }, []);

  useEffect(() => {
    if (!gridHost.current || !workspaceHost.current) return;
    const instance = GridStack.init({ column: 24, cellHeight: 40, margin: 4, animate: false,
      handle: '.panel-title', disableDrag: true, disableResize: true,
      columnOpts: { columnMax: 24, breakpoints: [{ w: 640, c: 1 }, { w: 1168, c: 12, layout: 'move' }] },
      resizable: { handles: 'e,s' } }, gridHost.current);
    if (!instance) return;
    grid.current = instance;
    const columns = instance.getColumn();
    instance.column(24);
    try { instance.load(validatedLayout(JSON.parse(localStorage.getItem(LAYOUT_KEY) ?? 'null')), false); }
    catch { instance.load(DEFAULT_LAYOUT, false); setLayoutError('Layout storage unavailable'); }
    instance.column(columns, columns === 12 ? 'move' : 'moveScale');
    const fit = () => {
      if (!workspaceHost.current) return;
      const current = validatedLayout(instance.save(false, false, undefined, instance.getColumn()), instance.getColumn());
      const cellHeight = viewportCellHeight(current, workspaceHost.current.clientHeight - 16);
      instance.cellHeight(cellHeight);
      setLayout(current); setGridSize({ columns: instance.getColumn(), cellHeight });
    };
    fitViewport.current = fit;
    fit();
    const save = () => {
      const value = validatedLayout(instance.save(false, false, undefined, 24));
      fit();
      if (fittingLayout.current) return;
      try { localStorage.setItem(LAYOUT_KEY, JSON.stringify(value)); setLayoutError(''); }
      catch { setLayoutError('Layout not saved'); }
    };
    instance.on('change', save);
    const observer = new ResizeObserver(fit);
    observer.observe(workspaceHost.current);
    return () => { observer.disconnect(); instance.off('change'); instance.destroy(false); grid.current = null; fitViewport.current = () => {}; };
  }, []);
  useEffect(() => {
    grid.current?.enableMove(Boolean(canResize)).enableResize(false);
  }, [canResize]);

  useEffect(() => {
    if (!settingsDirty || !savedSettings || !canEdit || settingsSaving || saveError || !angleValid) return;
    const timer = setTimeout(async () => {
      setSettingsSaving(true);
      try {
        const response = await api<Settings>('/api/settings', { method: 'PUT',
          headers: { 'X-Control-Lease': lease! },
          body: JSON.stringify({ expected_revision: savedSettings.revision, values: settingsDraft }) });
        setSavedSettings(response);
        await reload();
      } catch (failure) { setSaveError((failure as Error).message); }
      finally { setSettingsSaving(false); }
    }, 400);
    return () => clearTimeout(timer);
  }, [settingsDirty, savedSettings, settingsDraft, canEdit, lease, settingsSaving, saveError, angleValid, reload]);

  async function ownership() {
    setBusy(true); setError('');
    try {
      if (lease) { await api('/api/control/release', { method: 'POST', headers: { 'X-Control-Lease': lease } }); setLease(null); }
      else { const result = await api<{ lease: string }>('/api/control/claim', { method: 'POST' }); setLease(result.lease); }
      await reload();
    } catch (failure) { setError((failure as Error).message); }
    finally { setBusy(false); }
  }
  async function command(type: 'zone_add' | 'zone_delete') {
    if (!zones || !packet || !canEdit) return;
    setBusy(true); setError('');
    try {
      const result = await api<{ zones: Zones }>('/api/commands', { method: 'POST', headers: { 'X-Control-Lease': lease! },
        body: JSON.stringify({ id: crypto.randomUUID(), type, map_id: packet.info.map_id, expected_revision: zones.revision,
          ...(type === 'zone_add' ? { corners: rectangleCorners(draft[0], draft[1], packet.info.display.origin) } : { zone_id: selected }) }) });
      setZones(result.zones); setDraft([]); setDrawing(false); setSelected(null);
    } catch (failure) { setError((failure as Error).message); await reload().catch(() => {}); }
    finally { setBusy(false); }
  }
  async function stop() {
    setDrawing(false); setDraft([]); setLayoutEditing(false);
    try { await api('/api/stop', { method: 'POST' }); setError(''); await reload(); }
    catch (failure) { setError((failure as Error).message); }
  }
  async function simulationCommand(type: 'navigate' | 'cancel' | 'demo_reset') {
    if (!zones || !packet || !lease || !connected || (type !== 'cancel' && !canEdit)) return;
    setBusy(true); setError(''); setDrawing(false); setDraft([]); setLayoutEditing(false);
    try {
      await api('/api/commands', { method: 'POST', headers: { 'X-Control-Lease': lease },
        body: JSON.stringify({ id: crypto.randomUUID(), type, map_id: packet.info.map_id,
          frame: packet.info.frame, expected_revision: zones.revision, ...(type === 'navigate' ? { goal } : {}) }) });
      if (type === 'demo_reset') { setGoalText(['', '', '0']); setFocusRobotVersion(v => v + 1); }
    } catch (failure) { setError((failure as Error).message); }
    finally { await reload().catch(() => {}); setBusy(false); }
  }
  function previewGoal(point: Point) {
    if (!canLayout || drawing || busy) return;
    setGoalText(old => [point[0].toFixed(2), point[1].toFixed(2), old[2]]);
  }
  function startDrawing() { setSelected(null); setDraft([]); setDrawing(true); setLayoutEditing(false); }
  function resetLayout() {
    if (!canLayout || !grid.current) return;
    setMaximized(null); draggingDivider.current = null;
    const columns = grid.current.getColumn();
    fittingLayout.current = true;
    try { grid.current.column(24).load(DEFAULT_LAYOUT, false).column(columns, columns === 12 ? 'move' : 'moveScale'); }
    finally { fittingLayout.current = false; }
    fitViewport.current();
    try { localStorage.setItem(LAYOUT_KEY, JSON.stringify(validatedLayout(DEFAULT_LAYOUT))); setLayoutError(''); }
    catch { setLayoutError('Layout not saved'); }
  }
  function updateSetting(values: Settings['values']) { setSaveError(''); setSettingsDraft(values); }
  function applyDivider(tiles: Tile[], seam: Divider, delta: number, cellWidth: number, cellHeight: number) {
    const instance = grid.current;
    if (!instance) return;
    const columns = instance.getColumn();
    const previous = validatedLayout(instance.save(false, false, undefined, columns), columns);
    const next = moveDivider(tiles, seam, delta, cellWidth, cellHeight);
    const canonical = canonicalLayout(validatedLayout(instance.save(false, false, undefined, 24)), previous, next, columns);
    instance.load(next, false);
    // load() suppresses responsive cache updates; keep the desktop geometry in sync.
    if (columns < 24) instance.engine.cacheLayout(canonical, 24, true);
    fitViewport.current();
    try { localStorage.setItem(LAYOUT_KEY, JSON.stringify(canonical)); setLayoutError(''); }
    catch { setLayoutError('Layout not saved'); }
  }
  function resizePanel(id: string, dimension: 'w' | 'h', value: number) {
    if (!canResize || !grid.current || !Number.isInteger(value)) return;
    const tile = layout.find(t => t.id === id)!;
    const seam = dividers(layout).find(s => s.axis === (dimension === 'w' ? 'x' : 'y') && (s.before.includes(id) || s.after.includes(id)));
    if (seam && gridHost.current) applyDivider(layout, seam, (value - tile[dimension]) * (seam.before.includes(id) ? 1 : -1),
      gridHost.current.clientWidth / gridSize.columns, gridSize.cellHeight);
  }
  function beginDivider(event: PointerEvent<HTMLButtonElement>, seam: Divider) {
    if (!canResize || event.button !== 0 || !gridHost.current) return;
    event.preventDefault(); event.currentTarget.focus(); event.currentTarget.setPointerCapture(event.pointerId);
    draggingDivider.current = { seam, tiles: layout, start: seam.axis === 'x' ? event.clientX : event.clientY,
      cellWidth: gridHost.current.clientWidth / gridSize.columns, cellHeight: gridSize.cellHeight };
  }
  function dragDivider(event: PointerEvent<HTMLButtonElement>) {
    const drag = draggingDivider.current;
    if (!drag || !canResize) return;
    const distance = (drag.seam.axis === 'x' ? event.clientX : event.clientY) - drag.start;
    applyDivider(drag.tiles, drag.seam, distance / (drag.seam.axis === 'x' ? drag.cellWidth : drag.cellHeight), drag.cellWidth, drag.cellHeight);
  }
  function panel(id: string, content: ReactNode, tools?: ReactNode) {
    const defaults = DEFAULT_LAYOUT.find(item => item.id === id)!;
    return <div key={id} className={`grid-stack-item ${maximized === id ? 'maximized' : ''}`} gs-id={id}
      gs-x={defaults.x} gs-y={defaults.y} gs-w={defaults.w} gs-h={defaults.h} gs-min-w={defaults.minW} gs-min-h={defaults.minH}>
      <section className="grid-stack-item-content panel" aria-label={names[id]}>
        <header className="panel-header"><span className="panel-title">{names[id]}</span><div className="panel-tools">{tools}
          <Button variant="ghost" className="icon" aria-label={`${maximized === id ? 'Restore' : 'Maximize'} ${names[id]}`}
            onClick={() => setMaximized(maximized === id ? null : id)}>{maximized === id ? <Shrink size={15} /> : <Expand size={15} />}</Button>
        </div></header>{content}
      </section>
    </div>;
  }

  return <div className="dark app">
    <header className="topbar">
      <Dropdown.Root><Dropdown.Trigger asChild><Button variant="ghost" className="icon" aria-label="Open navigation"><Menu size={18} /></Button></Dropdown.Trigger>
        <Dropdown.Portal><Dropdown.Content className="menu" align="start" sideOffset={6}>
          <Dropdown.Item className="menu-item" onSelect={() => { setSettingsOpen(true); }}><Settings2 size={16} /> Settings</Dropdown.Item>
          <Dropdown.Item className="menu-item" disabled={!canLayout} onSelect={() => setLayoutEditing(v => !v)}><LayoutGrid size={16} /> {layoutEditing ? 'Lock layout' : 'Edit layout'}</Dropdown.Item>
          <Dropdown.Item className="menu-item" disabled={!canLayout} onSelect={resetLayout}><RotateCcw size={16} /> Restore default layout</Dropdown.Item>
        </Dropdown.Content></Dropdown.Portal>
      </Dropdown.Root>
      <strong className="brand">MYAGV CONTROL</strong><span className="demo-tag">DEMO</span>
      <Button variant="ghost" className="icon" aria-label="Restore default layout" title="Restore default layout" disabled={!canLayout} onClick={resetLayout}><RotateCcw size={16} /></Button>
      <span className={`connection ${connected ? 'online' : ''}`}>{connected ? 'Connected' : 'Offline'}</span>
      <span className="ownership">{lease ? 'Operator' : 'Observer'}</span>
      <Button onClick={() => void ownership()} disabled={!connected || busy || (!lease && Boolean(state?.operator_active))}>{lease ? 'Release' : 'Take'}</Button>
      <Button variant="danger" onClick={() => void stop()} className="stop">STOP</Button>
    </header>
    {error && <div role="alert" className="error-bar"><span>{error}</span><Button variant="ghost" className="icon" aria-label="Dismiss error" onClick={() => setError('')}><X size={15} /></Button></div>}
    <div className="workspace" ref={workspaceHost}>
    <main className={`grid-stack ${layoutEditing ? 'layout-editing' : ''}`} ref={gridHost}>
      {panel('map', <>
        <div className="map-tools">
          <span className="muted">{drawing ? draft.length < 2 ? 'Select two corners' : 'Preview' : 'Demo map'}</span>
          <div className="toolbar-actions">
            {drawing && <><Button variant="default" disabled={draft.length !== 2 || busy || !canEdit} onClick={() => void command('zone_add')}>Save</Button>
              <Button onClick={() => { setDrawing(false); setDraft([]); }}>Cancel</Button></>}
            {selected && !drawing && <Button disabled={!canEdit || busy} onClick={() => void command('zone_delete')}><Trash2 size={14} /> Delete</Button>}
            <Button variant="ghost" className="icon" aria-label="Fit map" onClick={() => setFitVersion(v => v + 1)}><Crosshair size={16} /></Button>
            <Dropdown.Root><Dropdown.Trigger asChild><Button variant="ghost" className="icon" aria-label="Map actions"><MoreHorizontal size={18} /></Button></Dropdown.Trigger>
              <Dropdown.Portal><Dropdown.Content className="menu" align="end" sideOffset={5}>
                <Dropdown.Item className="menu-item" disabled={!canEdit || !packet || busy} onSelect={startDrawing}><Square size={15} /> No-go zone</Dropdown.Item>
                <Dropdown.Item className="menu-item" disabled={!navigation} onSelect={() => setFocusRobotVersion(v => v + 1)}><Crosshair size={15} /> Fit robot</Dropdown.Item>
                <Dropdown.CheckboxItem className="menu-item" checked={showCostmap} onCheckedChange={setShowCostmap}><Square size={15} /> Global costmap {showCostmap ? '✓' : ''}</Dropdown.CheckboxItem>
                <Dropdown.Item className="menu-item" disabled={!canEdit || busy} onSelect={() => void simulationCommand('demo_reset')}><RotateCcw size={15} /> Reset demo pose</Dropdown.Item>
                <Dropdown.Item className="menu-item" onSelect={() => setSettingsOpen(true)}><Settings2 size={15} /> Settings</Dropdown.Item>
              </Dropdown.Content></Dropdown.Portal>
            </Dropdown.Root>
          </div>
        </div>
        {packet ? <MapView info={packet.info} image={packet.image} zones={zones?.zones ?? []} selected={selected}
          drawing={drawing && canEdit} draft={draft} fitVersion={fitVersion} onDraft={setDraft} onSelect={setSelected}
          navigation={navigation} goal={goal} connected={connected} showCostmap={showCostmap} focusRobotVersion={focusRobotVersion}
          onGoal={previewGoal} /> : <div className="empty">Loading map…</div>}
        {drawing && <details className="coordinate-editor"><summary>Coordinates · view meters</summary>
          <div className="coordinate-row">{['U1', 'V1', 'U2', 'V2'].map((label, i) => <label key={label}>{label}<input type="number" step="0.05"
            value={keyboardCorners[i]} onChange={e => setKeyboardCorners(v => v.map((item, n) => n === i ? e.target.value : item))} /></label>)}
            <Button disabled={!keyboardCorners.every(v => v.trim() !== '' && Number.isFinite(Number(v)))} onClick={() => {
              const [u1, v1, u2, v2] = keyboardCorners.map(Number); setDraft([[v1, u1], [v2, u2]]);
            }}>Preview</Button>
          </div>
        </details>}
      </>)}
      {panel('front', <div className="camera-body"><Camera size={26} strokeWidth={1} /><span>Camera unavailable</span></div>)}
      {panel('arm', <div className="camera-body"><Camera size={26} strokeWidth={1} /><span>Camera unavailable</span></div>)}
      {panel('operate', <div className="operate-body">
        <span className="subheading">POSE · DEMO</span>
        <output className="pose-readout" aria-label="Simulated map-frame pose">
          <span>X <b>{navigation?.pose.x_m.toFixed(2) ?? '—'}</b> m</span>
          <span>Y <b>{navigation?.pose.y_m.toFixed(2) ?? '—'}</b> m</span>
          <span>θ <b>{navigation ? (navigation.pose.yaw_rad * 180 / Math.PI).toFixed(0) : '—'}</b>°</span>
        </output>
        <span className="subheading">GOAL</span>
        <div className="goal-inputs">{['X (m)', 'Y (m)', 'θ (°)'].map((label, i) => <label key={label}>{label}<input type="number" step={i === 2 ? '5' : '.05'}
          min={i === 2 ? -180 : undefined} max={i === 2 ? 180 : undefined} placeholder={i === 2 ? '0' : 'Map click'} value={goalText[i]}
          disabled={!canLayout || busy} onChange={e => setGoalText(old => old.map((v, n) => n === i ? e.target.value : v))} /></label>)}</div>
        <div className="action-line"><Button variant="default" disabled={!canEdit || !goalValid || !costmapReady || busy}
          onClick={() => void simulationCommand('navigate')}>Simulate</Button><Button variant="default" disabled>Fetch</Button>
          <Button disabled={!connected || !lease || state?.phase === 'idle' || busy} onClick={() => void simulationCommand('cancel')}>Cancel</Button></div>
        <div className="section-divider" />
        <span className="subheading">NO-GO ZONES</span>
        <div className="zone-list">{zones?.zones.length ? zones.zones.map((zone, i) => <button key={zone.id}
          className={`zone-row ${selected === zone.id ? 'selected' : ''}`} onClick={() => { setSelected(zone.id); setDrawing(false); setDraft([]); }}>
          <Square size={14} /> Zone {i + 1}<span className="row-end">{selected === zone.id ? 'Selected' : ''}</span>
        </button>) : <span className="muted">None</span>}</div>
        <span className="muted enforcement">{costmapReady ? 'Demo costmap applied' : 'Costmap pending'}</span>
        <div className="section-divider" /><span className="subheading">TASK</span><span className="muted" role="status">{connected ? navigation?.status ?? 'Loading…' : 'Offline'}</span>
      </div>)}
    </main>
    <div className="panel-dividers">
      {!maximized && dividers(layout).map(seam => {
        const vertical = seam.axis === 'x';
        return <button key={`${seam.axis}:${seam.before.join(',')}:${seam.after.join(',')}`}
          className={`panel-divider ${vertical ? 'vertical' : 'horizontal'}`} role="separator"
          aria-orientation={vertical ? 'vertical' : 'horizontal'}
          aria-label={`Resize ${seam.before.map(id => names[id]).join(' / ')} and ${seam.after.map(id => names[id]).join(' / ')}`}
          aria-valuenow={seam.position} aria-valuemin={0} aria-valuemax={vertical ? gridSize.columns : Math.max(...layout.map(t => t.y + t.h))}
          disabled={!canResize} title={canResize ? 'Drag to resize adjacent panels' : 'Unlock Edit layout to resize'}
          style={vertical ? { left: `calc(${seam.position / gridSize.columns * 100}% - 4px)`, top: seam.start * gridSize.cellHeight + 4,
            height: (seam.end - seam.start) * gridSize.cellHeight - 8 } :
            { top: seam.position * gridSize.cellHeight - 4, left: `calc(${seam.start / gridSize.columns * 100}% + 4px)`,
              width: `calc(${(seam.end - seam.start) / gridSize.columns * 100}% - 8px)` }}
          onPointerDown={event => beginDivider(event, seam)} onPointerMove={dragDivider}
          onPointerUp={event => { draggingDivider.current = null; if (event.currentTarget.hasPointerCapture(event.pointerId)) event.currentTarget.releasePointerCapture(event.pointerId); }}
          onLostPointerCapture={() => { draggingDivider.current = null; }}
          onKeyDown={event => {
            const delta = (vertical ? { ArrowLeft: -1, ArrowRight: 1 } : { ArrowUp: -1, ArrowDown: 1 })[event.key as 'ArrowLeft'];
            if (delta && canResize && gridHost.current) {
              event.preventDefault(); applyDivider(layout, seam, delta * (event.shiftKey ? 2 : 1),
                gridHost.current.clientWidth / gridSize.columns, gridSize.cellHeight);
            }
          }} />;
      })}
    </div>
    </div>
    <Dialog.Root open={settingsOpen} onOpenChange={setSettingsOpen}>
      <Dialog.Portal><Dialog.Overlay className="dialog-overlay" /><Dialog.Content className="settings-dialog">
        <header className="dialog-header"><Dialog.Title>SETTINGS</Dialog.Title><Dialog.Close asChild><Button variant="ghost" className="icon" aria-label="Close settings"><X size={18} /></Button></Dialog.Close></header>
        <Dialog.Description className="sr-only">Map display and workspace settings</Dialog.Description>
        <div className="settings-body">
          <span className="subheading">MAP DISPLAY</span>
          <label className="setting-row">Auto-align walls<input type="checkbox" checked={settingsDraft.auto_align} disabled={!canEdit}
            onChange={e => updateSetting({ ...settingsDraft, auto_align: e.target.checked })} /></label>
          <label className="setting-row">Display angle (°)<input type="number" min="-180" max="180" step="1" value={angleText} aria-invalid={!angleValid}
            disabled={!canEdit || settingsDraft.auto_align} onChange={e => {
              setAngleText(e.target.value);
              const angle = Number(e.target.value); if (e.target.value.trim() !== '' && Number.isFinite(angle) && angle >= -180 && angle <= 180) updateSetting({ ...settingsDraft, manual_angle_deg: angle });
            }} /></label>
          {packet && settingsDraft.auto_align && !packet.info.display.alignment_available && <span className="muted">Wall direction unavailable</span>}
          <div className="save-state" role="status">{!angleValid ? 'Angle must be −180…180°' : saveError || (settingsSaving ? 'Saving…' : settingsDirty ? 'Unsaved' : savedSettings ? 'Saved' : 'Loading…')}
            {saveError && <Button onClick={() => setSaveError('')} disabled={!canEdit}>Retry</Button>}</div>
          <div className="section-divider" /><span className="subheading">WORKSPACE</span>
          <label className="setting-row">Edit layout<input type="checkbox" checked={layoutEditing} disabled={!canLayout} onChange={e => setLayoutEditing(e.target.checked)} /></label>
          {layout.map(widget => <div className="size-row" key={widget.id}><span>{names[widget.id!]}</span>
            <label>W<input type="number" min="1" max={grid.current?.getColumn() ?? 24} value={widget.w} disabled={!canResize || !dividers(layout).some(s => s.axis === 'x' && (s.before.includes(widget.id) || s.after.includes(widget.id)))}
              onChange={e => resizePanel(widget.id!, 'w', Number(e.target.value))} /></label>
            <label>H<input type="number" min={widget.minH} max="40" value={widget.h} disabled={!canResize || !dividers(layout).some(s => s.axis === 'y' && (s.before.includes(widget.id) || s.after.includes(widget.id)))}
              onChange={e => resizePanel(widget.id!, 'h', Number(e.target.value))} /></label></div>)}
          <Button onClick={resetLayout} disabled={!canLayout}>Restore default layout</Button>{layoutError && <span role="alert" className="muted">{layoutError}</span>}
        </div>
        <footer className="dialog-footer"><span className="muted">{canEdit ? 'Auto-save' : 'Take control to edit'}</span><Button variant="danger" onClick={() => void stop()}>STOP</Button></footer>
      </Dialog.Content></Dialog.Portal>
    </Dialog.Root>
  </div>;
}
