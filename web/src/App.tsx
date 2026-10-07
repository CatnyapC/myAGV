import { useCallback, useEffect, useRef, useState, type ReactNode } from 'react';
import * as Dialog from '@radix-ui/react-dialog';
import * as Dropdown from '@radix-ui/react-dropdown-menu';
import { GridStack, type GridStackWidget } from 'gridstack';
import { Camera, Crosshair, Expand, LayoutGrid, Menu, MoreHorizontal, RotateCcw, Settings2, Shrink, Square, Trash2, X } from 'lucide-react';
import { MapView, rectangleCorners } from './MapView';
import { type MapInfo, type Point, type Settings, type Zones } from './mapGeometry';
import { Button } from './ui';

type State = { robot_id: string; demo: boolean; phase: string; operator_active: boolean };
const LAYOUT_KEY = 'myagv:local-demo:layout:v1';
const DEFAULT_LAYOUT: GridStackWidget[] = [
  { id: 'map', x: 0, y: 0, w: 12, h: 12, minW: 8, minH: 9 },
  { id: 'front', x: 12, y: 0, w: 6, h: 6, minW: 6, minH: 6 },
  { id: 'arm', x: 12, y: 6, w: 6, h: 6, minW: 6, minH: 6 },
  { id: 'operate', x: 18, y: 0, w: 6, h: 12, minW: 6, minH: 11 },
];
const names: Record<string, string> = { map: 'MAP', front: 'FRONT CAMERA', arm: 'ARM CAMERA', operate: 'OPERATE' };

function validatedLayout(value: unknown): GridStackWidget[] {
  if (!Array.isArray(value) || value.length !== 4) return DEFAULT_LAYOUT;
  const ids = new Set();
  const clean: GridStackWidget[] = [];
  for (const item of value) {
    const defaults = DEFAULT_LAYOUT.find(w => w.id === item?.id);
    if (!defaults || ids.has(item.id)) return DEFAULT_LAYOUT;
    ids.add(item.id);
    const { x, y, w, h } = item;
    if (![x, y, w, h].every(Number.isInteger) || x < 0 || y < 0 || w < 1 || h < (defaults.minH ?? 1) || x + w > 24 || y > 100 || h > 40) return DEFAULT_LAYOUT;
    clean.push({ ...defaults, x, y, w, h });
  }
  return clean;
}

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
  const [settingsOpen, setSettingsOpen] = useState(false);
  const [layoutEditing, setLayoutEditing] = useState(false);
  const [layoutError, setLayoutError] = useState('');
  const [maximized, setMaximized] = useState<string | null>(null);
  const [layout, setLayout] = useState<GridStackWidget[]>(DEFAULT_LAYOUT);
  const [keyboardCorners, setKeyboardCorners] = useState(['1', '1', '3', '3']);
  const gridHost = useRef<HTMLDivElement>(null);
  const grid = useRef<GridStack | null>(null);
  const generation = useRef(0);
  const previousMap = useRef('');
  const editingSettings = useRef(false);
  const settingsDirty = savedSettings !== null && JSON.stringify(settingsDraft) !== JSON.stringify(savedSettings.values);
  const angleValid = angleText.trim() !== '' && Number.isFinite(Number(angleText)) && Math.abs(Number(angleText)) <= 180;
  editingSettings.current = settingsDirty || settingsSaving || !angleValid;
  const canEdit = Boolean(lease && connected && state?.phase === 'idle');

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
      if (!previousMap.current.startsWith(`${info.map_id}:`)) setSelected(null);
      previousMap.current = key;
      setPacket(old => {
        // Keep map/pan intact on zone or lease events; replace raster only on a new view.
        if (old && old.info.map_id === info.map_id && old.info.revision === info.revision && old.info.display.view_revision === info.display.view_revision) {
          URL.revokeObjectURL(image); return old;
        }
        return { info, image };
      });
      setState(nextState); setZones(nextZones); setSavedSettings(preferences);
      if (!editingSettings.current) { setSettingsDraft(preferences.values); setAngleText(String(preferences.values.manual_angle_deg)); }
      setConnected(true);
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
        socket.onmessage = () => { void reload().catch(failure => { setError(failure.message); }); };
        socket.onclose = () => {
          if (!disposed) { setConnected(false); setLease(null); setDrawing(false); setDraft([]); timer = setTimeout(connect, 1500); }
        };
        socket.onerror = () => socket?.close();
      } catch (failure) {
        if (!disposed) { setConnected(false); setError((failure as Error).message); timer = setTimeout(connect, 1500); }
      }
    };
    void connect();
    return () => { disposed = true; clearTimeout(timer); socket?.close(); ++generation.current; };
  }, [reload]);

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
    if (!canEdit) { setDrawing(false); setDraft([]); setLayoutEditing(false); }
  }, [canEdit]);
  useEffect(() => {
    const key = (event: KeyboardEvent) => { if (event.key === 'Escape') { setDrawing(false); setDraft([]); setMaximized(null); } };
    window.addEventListener('keydown', key);
    return () => window.removeEventListener('keydown', key);
  }, []);

  useEffect(() => {
    if (!gridHost.current) return;
    const instance = GridStack.init({ column: 24, cellHeight: 40, margin: 4, animate: false,
      handle: '.panel-title', disableDrag: true, disableResize: true,
      columnOpts: { columnMax: 24, breakpoints: [{ w: 640, c: 1 }, { w: 1120, c: 12 }] },
      resizable: { handles: 'e,se,s,sw,w' } }, gridHost.current);
    if (!instance) return;
    grid.current = instance;
    try { instance.load(validatedLayout(JSON.parse(localStorage.getItem(LAYOUT_KEY) ?? 'null')), false); }
    catch { instance.load(DEFAULT_LAYOUT, false); setLayoutError('Layout storage unavailable'); }
    setLayout(validatedLayout(instance.save(false, false, undefined, 24)));
    const save = () => {
      const value = validatedLayout(instance.save(false, false, undefined, 24));
      setLayout(value);
      try { localStorage.setItem(LAYOUT_KEY, JSON.stringify(value)); setLayoutError(''); }
      catch { setLayoutError('Layout not saved'); }
    };
    instance.on('change', save);
    return () => { instance.off('change'); instance.destroy(false); grid.current = null; };
  }, []);
  useEffect(() => {
    const enabled = canEdit && layoutEditing && !maximized;
    grid.current?.enableMove(enabled).enableResize(enabled);
  }, [canEdit, layoutEditing, maximized]);

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
    try { const result = await api<{ reason: string }>('/api/stop', { method: 'POST' }); setError(result.reason); }
    catch (failure) { setError((failure as Error).message); }
  }
  function startDrawing() { setSelected(null); setDraft([]); setDrawing(true); setLayoutEditing(false); }
  function resetLayout() { grid.current?.load(DEFAULT_LAYOUT, false); }
  function updateSetting(values: Settings['values']) { setSaveError(''); setSettingsDraft(values); }
  function resizePanel(id: string, dimension: 'w' | 'h', value: number) {
    if (!canEdit || !layoutEditing || !grid.current || !Number.isInteger(value)) return;
    const widget = gridHost.current?.querySelector(`[gs-id="${id}"]`) as HTMLElement | null;
    if (widget) grid.current.update(widget, { [dimension]: value });
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
          <Dropdown.Item className="menu-item" disabled={!canEdit} onSelect={() => setLayoutEditing(v => !v)}><LayoutGrid size={16} /> {layoutEditing ? 'Lock layout' : 'Edit layout'}</Dropdown.Item>
          <Dropdown.Item className="menu-item" disabled={!canEdit} onSelect={resetLayout}><RotateCcw size={16} /> Reset layout</Dropdown.Item>
        </Dropdown.Content></Dropdown.Portal>
      </Dropdown.Root>
      <strong className="brand">MYAGV CONTROL</strong><span className="demo-tag">DEMO</span>
      <span className={`connection ${connected ? 'online' : ''}`}>{connected ? 'Connected' : 'Offline'}</span>
      <span className="ownership">{lease ? 'Operator' : 'Observer'}</span>
      <Button onClick={() => void ownership()} disabled={!connected || busy || (!lease && Boolean(state?.operator_active))}>{lease ? 'Release' : 'Take'}</Button>
      <Button variant="danger" onClick={() => void stop()} className="stop">STOP</Button>
    </header>
    {error && <div role="alert" className="error-bar"><span>{error}</span><Button variant="ghost" className="icon" aria-label="Dismiss error" onClick={() => setError('')}><X size={15} /></Button></div>}
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
                <Dropdown.Item className="menu-item" onSelect={() => setSettingsOpen(true)}><Settings2 size={15} /> Settings</Dropdown.Item>
              </Dropdown.Content></Dropdown.Portal>
            </Dropdown.Root>
          </div>
        </div>
        {packet ? <MapView info={packet.info} image={packet.image} zones={zones?.zones ?? []} selected={selected}
          drawing={drawing && canEdit} draft={draft} fitVersion={fitVersion} onDraft={setDraft} onSelect={setSelected} /> : <div className="empty">Loading map…</div>}
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
        <label className="subheading" htmlFor="target">TARGET</label><input id="target" placeholder="Select an item" disabled />
        <div className="action-line"><Button variant="default" disabled>Go</Button><Button variant="default" disabled>Fetch</Button><Button disabled>Cancel</Button></div>
        <div className="section-divider" />
        <span className="subheading">NO-GO ZONES</span>
        <div className="zone-list">{zones?.zones.length ? zones.zones.map((zone, i) => <button key={zone.id}
          className={`zone-row ${selected === zone.id ? 'selected' : ''}`} onClick={() => { setSelected(zone.id); setDrawing(false); setDraft([]); }}>
          <Square size={14} /> Zone {i + 1}<span className="row-end">{selected === zone.id ? 'Selected' : ''}</span>
        </button>) : <span className="muted">None</span>}</div>
        <span className="muted enforcement">Navigation enforcement unavailable</span>
        <div className="section-divider" /><span className="subheading">TASK</span><span className="muted">Hardware unavailable</span>
      </div>)}
    </main>
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
          <label className="setting-row">Edit layout<input type="checkbox" checked={layoutEditing} disabled={!canEdit} onChange={e => setLayoutEditing(e.target.checked)} /></label>
          {layout.map(widget => <div className="size-row" key={widget.id}><span>{names[widget.id!]}</span>
            <label>W<input type="number" min="1" max={grid.current?.getColumn() ?? 24} value={widget.w} disabled={!canEdit || !layoutEditing}
              onChange={e => resizePanel(widget.id!, 'w', Number(e.target.value))} /></label>
            <label>H<input type="number" min={widget.minH} max="40" value={widget.h} disabled={!canEdit || !layoutEditing}
              onChange={e => resizePanel(widget.id!, 'h', Number(e.target.value))} /></label></div>)}
          <Button onClick={resetLayout} disabled={!canEdit}>Reset layout</Button>{layoutError && <span role="alert" className="muted">{layoutError}</span>}
        </div>
        <footer className="dialog-footer"><span className="muted">{canEdit ? 'Auto-save' : 'Take control to edit'}</span><Button variant="danger" onClick={() => void stop()}>STOP</Button></footer>
      </Dialog.Content></Dialog.Portal>
    </Dialog.Root>
  </div>;
}
