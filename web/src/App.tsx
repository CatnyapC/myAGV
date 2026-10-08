import { useCallback, useEffect, useRef, useState, type ReactNode, type PointerEvent } from 'react';
import * as Dialog from '@radix-ui/react-dialog';
import * as Dropdown from '@radix-ui/react-dropdown-menu';
import { GridStack } from 'gridstack';
import { Camera, Crosshair, Dice5, Expand, Hand, LayoutGrid, LoaderCircle, Menu, MoreHorizontal, Play, Plus, RotateCcw, Search, Settings2, Shrink, Square, Trash2, X } from 'lucide-react';
import { MapView, rectangleCorners } from './MapView';
import { type ItemLocation, type MapInfo, type Navigation, type Origin, type PhotoIndex, type Point, type Settings, type Stations, type Zones } from './mapGeometry';
import { Button } from './ui';
import { BatteryStatus } from './BatteryStatus';
import { api, commandId, hardwareBlocks, waitForControlLease } from './api';
import { HardwareControls, MapUpdateControls, type HardwareState, type HeldInput } from './HardwareControls';
import { PhotoEditor, StoredPhoto } from './PhotoEditor';
import { FetchTest } from './FetchTest';
import { DEFAULT_LAYOUT, canonicalLayout, dividers, moveDivider, validatedLayout, viewportCellHeight, type Divider, type Tile } from './layout';

type State = { hardware?: HardwareState; robot_id: string; demo: boolean; phase: string; navigation: Navigation; cameras?: Partial<Record<'front' | 'arm', { url: string; stamp_s: number }>>; llm: { model: string; status: string; reasoning_effort: string } };
type Resolution = { status: 'matched' | 'ambiguous' | 'not_found'; item_ids: string[]; index_revision: number; settings_revision: number; map_id: string; photo_id?: string; goal?: Origin; blocked_reason?: string; target_source?: 'estimated' };
const LAYOUT_KEY = 'myagv:local-demo:layout:v2';
const names: Record<string, string> = { map: 'MAP', front: 'FRONT CAMERA', arm: 'ARM CAMERA', operate: 'OPERATE' };
const REQUEST_EXAMPLES = [
  '塩の瓶を持ってきてください。',
  '塩の瓶をこちらに運んでもらえますか。',
  '塩の瓶を取ってきて。',
  '塩の瓶をここまで届けてください。',
  '醤油の瓶を持ってきてください。',
  '醤油の瓶をこちらに運んでもらえますか。',
  '醤油の瓶を取ってきて。',
  '醤油の瓶をここまで届けてください。',
  'コーヒーの瓶を持ってきてください。',
  'コーヒーの瓶をこちらに運んでもらえますか。',
  'コーヒーの瓶を取ってきて。',
  'コーヒーの瓶をここまで届けてください。',
  'お茶の瓶を持ってきてください。',
  'お茶の瓶をこちらに運んでもらえますか。',
  'お茶の瓶を取ってきて。',
  'お茶の瓶をここまで届けてください。',
];

export function App() {
  const [state, setState] = useState<State | null>(null);
  const [connected, setConnected] = useState(false);
  const [obstacleSize, setObstacleSize] = useState<number | null>(null);
  const [packet, setPacket] = useState<{ info: MapInfo; image: string } | null>(null);
  const [zones, setZones] = useState<Zones | null>(null);
  const [savedSettings, setSavedSettings] = useState<Settings | null>(null);
  const [settingsDraft, setSettingsDraft] = useState<Settings['values']>({ auto_align: true, manual_angle_deg: 0, llm_model: 'deepseek/deepseek-v4.1-flash', reasoning_effort: 'off' });
  const [angleText, setAngleText] = useState('0');
  const [settingsSaving, setSettingsSaving] = useState(false);
  const [saveError, setSaveError] = useState('');
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState('');
  const [drawing, setDrawing] = useState(false);
  const [draft, setDraft] = useState<Point[]>([]);
  const [selected, setSelected] = useState<string | null>(null);
  const [fitVersion, setFitVersion] = useState(0);
  const [focusRobotVersion, setFocusRobotVersion] = useState(0);
  const [showCostmap, setShowCostmap] = useState(false);
  const [goalText, setGoalText] = useState(['', '', '0']);
  const [photos, setPhotos] = useState<PhotoIndex | null>(null);
  const [stations, setStations] = useState<Stations>({});
  const [selectedPhotoId, setSelectedPhotoId] = useState<string | null>(null);
  const [selectedItemId, setSelectedItemId] = useState('');
  const [photoMode, setPhotoMode] = useState<'import' | 'edit' | 'uploaded' | null>(null);
  const [requestText, setRequestText] = useState('');
  const [resolution, setResolution] = useState<Resolution | null>(null);
  const [settingsOpen, setSettingsOpen] = useState(false);
  const [resetMapOpen, setResetMapOpen] = useState(false);
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
  const session = useRef('');
  const leaseAck = useRef(0);
  const actionEpoch = useRef(0);
  const held = useRef<HeldInput>(null);
  const hardwareMode = useRef(false);
  hardwareMode.current = state?.demo === false;
  const previousMap = useRef('');
  const editingSettings = useRef(false);
  const resolutionRequest = useRef<AbortController | null>(null);
  const resolutionContext = useRef('');
  resolutionContext.current = `${packet?.info.map_id}:${photos?.revision}:${savedSettings?.revision}`;
  const settingsDirty = savedSettings !== null && JSON.stringify(settingsDraft) !== JSON.stringify(savedSettings.values);
  const angleValid = angleText.trim() !== '' && Number.isFinite(Number(angleText)) && Math.abs(Number(angleText)) <= 180;
  editingSettings.current = settingsDirty || settingsSaving || !angleValid;
  const canEdit = connected && state?.phase === 'idle';
  const canEditPhotos = connected && (state?.phase === 'idle' || state?.phase === 'update_paused');
  const canLayout = canEdit;
  const canResize = canLayout && !maximized;
  const canMove = canResize && layoutEditing;
  const navigation = state?.navigation ?? null;
  const goalValid = goalText.every(v => v.trim() !== '' && Number.isFinite(Number(v))) && Math.abs(Number(goalText[2])) <= 180;
  const goal = goalValid ? { x_m: Number(goalText[0]), y_m: Number(goalText[1]), yaw_rad: Number(goalText[2]) * Math.PI / 180 } : null;
  const costmapReady = navigation?.costmap.ready && navigation.costmap.applied_zone_revision === zones?.revision;
  const selectedPhoto = photos?.photos.find(p => p.id === selectedPhotoId) ?? null;
  const locationPhotos = photos?.photos.filter(p => p.kind === 'observation' && p.map_id === packet?.info.map_id && p.current !== false) ?? [];
  const selectedItem = photos?.items.find(i => i.id === selectedItemId);
  const hardwareReasons = !connected ? ['UI disconnected'] : hardwareBlocks(state?.hardware);
  const goReasons = [...(!connected ? ['UI disconnected'] : hardwareBlocks(state?.hardware, false)), ...(!goalValid ? ['Choose a goal on the map or enter X/Y/heading'] : []), ...(busy ? ['Current request running'] : [])];
  const fetchReasons = [...hardwareReasons, ...(!selectedItem ? ['Select an item linked to a taught pickup station'] : !selectedItem.observation_current ? ['Confirm this item in a current observation'] : selectedItem.station_status !== 'ready' ? ['Teach a station and confirm its association in the item photo editor'] : state?.llm?.status === 'configured' && !selectedItem.grasp_goal_ready ? ['Record correct-grasp views in the item editor'] : []), ...(!state?.cameras?.arm ? ['Arm camera unavailable'] : []), ...(busy ? ['Current request running'] : [])];

  const reload = useCallback(async () => {
    const current = ++generation.current;
    const [nextState, info, nextZones, preferences, nextPhotos, nextStations] = await Promise.all([
      api<State>('/api/state'), api<MapInfo>('/api/map'), api<Zones>('/api/no-go-zones'), api<Settings>('/api/settings'),
      api<PhotoIndex>('/api/items'), api<Stations>('/api/stations'),
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
        // Keep map/pan intact on zone events; replace raster only on a new view.
        if (old && old.info.map_id === info.map_id && old.info.revision === info.revision && old.info.display.view_revision === info.display.view_revision) {
          URL.revokeObjectURL(image); return old;
        }
        return { info, image };
      });
      setState(old => old && old.navigation.stamp_s > nextState.navigation.stamp_s ? old : nextState);
      setZones(nextZones); setSavedSettings(preferences);
      setPhotos(nextPhotos); setStations(nextStations);
      if (!editingSettings.current) { setSettingsDraft(preferences.values); setAngleText(String(preferences.values.manual_angle_deg)); }
    } catch (failure) { URL.revokeObjectURL(image); throw failure; }
  }, []);

  useEffect(() => () => { if (packet) URL.revokeObjectURL(packet.image); }, [packet]);
  useEffect(() => () => resolutionRequest.current?.abort(), []);
  useEffect(() => {
    if (resolution && `${resolution.map_id}:${resolution.index_revision}:${resolution.settings_revision}` !== resolutionContext.current) {
      setResolution(null); setGoalText(['', '', '0']);
    }
  }, [resolution, photos?.revision, packet?.info.map_id, savedSettings?.revision]);
  useEffect(() => {
    if (selectedPhotoId && photos && !photos.photos.some(p => p.id === selectedPhotoId)) { setSelectedPhotoId(null); if (photoMode !== 'import') setPhotoMode(null); }
    if (selectedItemId && photos && !photos.items.some(i => i.id === selectedItemId)) setSelectedItemId('');
  }, [photos, selectedPhotoId, selectedItemId, photoMode]);

  useEffect(() => {
    let disposed = false, timer: ReturnType<typeof setTimeout>, socket: WebSocket | null = null;
    const connect = async () => {
      try {
        await reload();
        if (disposed) return;
        socket = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/api/events`);
        socket.onopen = () => setConnected(true);
        socket.onmessage = event => {
          const message = JSON.parse(event.data) as { type: string; state?: State; session_id?: string; nonce?: string };
          if (message.type === 'lease') {
            session.current = message.session_id ?? '';
            socket?.send(JSON.stringify({ nonce: message.nonce, input: held.current }));
            return;
          }
          if (message.type === 'lease_ack') { ++leaseAck.current; return; }
          if (message.type === 'telemetry' && message.state) {
            setState(message.state);
          } else void reload().catch(failure => { setError(failure.message); });
        };
        socket.onclose = () => {
          session.current = ''; held.current = null; ++actionEpoch.current;
          if (!disposed) { resolutionRequest.current?.abort(); setResolution(null); setConnected(false); setDrawing(false); setDraft([]); timer = setTimeout(connect, 1500); }
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
    const release = () => {
      ++actionEpoch.current;
      held.current = null;
    };
    const leave = () => {
      release();
      if (hardwareMode.current) void api('/api/stop', { method: 'POST', keepalive: true, body: JSON.stringify({ passive: true }) }).catch(() => {});
    };
    const visibility = () => { if (document.hidden) release(); };
    window.addEventListener('blur', release); window.addEventListener('pagehide', leave);
    document.addEventListener('visibilitychange', visibility);
    return () => { window.removeEventListener('blur', release); window.removeEventListener('pagehide', leave); document.removeEventListener('visibilitychange', visibility); };
  }, []);
  useEffect(() => {
    if (!canEdit) { resolutionRequest.current?.abort(); setDrawing(false); setDraft([]); }
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
    grid.current?.enableMove(Boolean(canMove)).enableResize(false);
  }, [canMove]);

  useEffect(() => {
    if (!settingsDirty || !savedSettings || !canEdit || settingsSaving || saveError || !angleValid) return;
    const timer = setTimeout(async () => {
      setSettingsSaving(true);
      try {
        const response = await api<Settings>('/api/settings', { method: 'PUT',
          body: JSON.stringify({ expected_revision: savedSettings.revision, values: settingsDraft }) });
        setSavedSettings(response);
        await reload();
      } catch (failure) { setSaveError((failure as Error).message); }
      finally { setSettingsSaving(false); }
    }, 400);
    return () => clearTimeout(timer);
  }, [settingsDirty, savedSettings, settingsDraft, canEdit, settingsSaving, saveError, angleValid, reload]);
  async function controlReady(timeoutMs = 5000) {
    if (hardwareMode.current) await waitForControlLease(() => ({ ack: leaseAck.current, epoch: actionEpoch.current, visible: !document.hidden }), timeoutMs);
  }
  async function command(type: 'zone_add' | 'zone_delete') {
    if (!zones || !packet || !canEdit) return;
    setBusy(true); setError('');
    try {
      await controlReady();
      const result = await api<{ zones: Zones }>('/api/commands', { method: 'POST',
        body: JSON.stringify({ id: commandId(), type, session_id: session.current, stop_epoch: state?.hardware?.stop_epoch, map_id: packet.info.map_id, expected_revision: zones.revision,
          ...(type === 'zone_add' ? { corners: rectangleCorners(draft[0], draft[1], packet.info.display.origin) } : { zone_id: selected }) }) });
      setZones(result.zones); setDraft([]); setDrawing(false); setSelected(null);
    } catch (failure) { setError((failure as Error).message); await reload().catch(() => {}); }
    finally { setBusy(false); }
  }
  async function hardwareCommand(type: string, values: Record<string, unknown> = {}) {
    if (!packet || !zones || !session.current) return;
    setError('');
    try {
      await controlReady(type === 'manual' ? 1000 : 5000);
      if (type === 'manual' && !held.current) return;
      await api('/api/commands', { method: 'POST', body: JSON.stringify({ ...values,
        type, id: commandId(), session_id: session.current, stop_epoch: state?.hardware?.stop_epoch, map_id: packet.info.map_id, expected_revision: zones.revision }) });
      if (type === 'teach' || type === 'transport_record' || type === 'reset_map') await reload();
    } catch (failure) {
      // Releasing a manual button aborts its pending start; a newer hold may already exist.
      if (type === 'manual' && session.current && failure instanceof DOMException && failure.name === 'AbortError') return;
      held.current = null; setError((failure as Error).message); if (type === 'reset_map') throw failure;
    }
  }
  async function stop() {
    ++actionEpoch.current;
    held.current = null;
    resolutionRequest.current?.abort(); setResolution(null);
    setDrawing(false); setDraft([]); setLayoutEditing(false);
    try { await api('/api/stop', { method: 'POST' }); await reload(); }
    catch (failure) { setError((failure as Error).message); }
  }
  async function simulationCommand(type: 'navigate' | 'cancel' | 'demo_reset') {
    if (!zones || !packet || !connected || (type !== 'cancel' && !canEdit)) return;
    setBusy(true); setError(''); setDrawing(false); setDraft([]); setLayoutEditing(false);
    try {
      await controlReady();
      await api('/api/commands', { method: 'POST',
        body: JSON.stringify({ id: commandId(), type, session_id: session.current, stop_epoch: state?.hardware?.stop_epoch, map_id: packet.info.map_id,
          frame: packet.info.frame, expected_revision: zones.revision, ...(type === 'navigate' ? { goal } : {}) }) });
      if (type === 'demo_reset') { setGoalText(['', '', '0']); setFocusRobotVersion(v => v + 1); }
    } catch (failure) { setError((failure as Error).message); }
    finally { await reload().catch(() => {}); setBusy(false); }
  }
  function previewGoal(point: Point) {
    if (!canLayout || drawing || busy) return;
    setGoalText(old => [point[0].toFixed(2), point[1].toFixed(2), old[2]]);
  }
  function selectPhoto(id: string) {
    resolutionRequest.current?.abort(); setResolution(null); setGoalText(['', '', '0']);
    setSelectedPhotoId(id); setSelected(null);
    setSelectedItemId(photos?.photos.find(p => p.id === id)?.item_id ?? '');
  }
  function selectItem(id: string) {
    resolutionRequest.current?.abort(); setResolution(null); setGoalText(['', '', '0']);
    setSelectedItemId(id);
    setSelectedPhotoId(photos?.photos.find(p => p.item_id === id && p.kind === 'observation' && p.map_matches && p.available)?.id ?? photos?.photos.find(p => p.item_id === id)?.id ?? null);
  }
  async function selectLocation(location: ItemLocation) {
    if (!canEdit || busy || !packet || !photos || !zones) return;
    const epoch = actionEpoch.current, mapId = packet.info.map_id;
    setBusy(true); setError(''); setResolution(null); setGoalText(['', '', '0']);
    setSelectedItemId(location.item_id); setSelectedPhotoId(location.photo_id); setSelected(null);
    try {
      const result = await api<{ goal: Origin; map_id: string }>('/api/item-locations/approach', { method: 'POST',
        body: JSON.stringify({ map_id: mapId, item_id: location.item_id, photo_id: location.photo_id,
          expected_revision: photos.revision, zone_revision: zones.revision }) });
      if (epoch !== actionEpoch.current || !previousMap.current.startsWith(`${result.map_id}:`)) return;
      setGoalText([result.goal.x_m.toFixed(3), result.goal.y_m.toFixed(3), (result.goal.yaw_rad * 180 / Math.PI).toFixed(1)]);
    } catch (failure) { setError((failure as Error).message); }
    finally { setBusy(false); }
  }
  async function estimatePositions() {
    if (!photos || !packet || busy || !canEditPhotos) return;
    setBusy(true); setError('');
    try {
      const index = await api<PhotoIndex>('/api/item-locations/estimate', { method: 'POST',
        body: JSON.stringify({ map_id: packet.info.map_id, expected_revision: photos.revision }) });
      setPhotos(index);
    } catch (failure) { setError((failure as Error).message); }
    finally { setBusy(false); }
  }
  function updateRequest(text: string) { setRequestText(text); setResolution(null); setGoalText(['', '', '0']); }
  async function resolveRequest() {
    if (!canEdit || busy || !photos || !packet || !requestText.trim() || state?.llm?.status !== 'configured' || settingsDirty || settingsSaving || saveError) return;
    const controller = new AbortController(); resolutionRequest.current = controller;
    setBusy(true); setError(''); setResolution(null); setGoalText(['', '', '0']); setSelectedItemId(''); setSelectedPhotoId(null);
    try {
      const result = await api<Resolution>('/api/resolve', { method: 'POST', signal: controller.signal,
        body: JSON.stringify({ text: requestText.trim(), map_id: packet.info.map_id, expected_revision: photos.revision }) });
      if (controller.signal.aborted || `${result.map_id}:${result.index_revision}:${result.settings_revision}` !== resolutionContext.current) return;
      setResolution(result); setSelected(null);
      const id = result.status === 'matched' ? result.item_ids[0] : '';
      setSelectedItemId(id); setSelectedPhotoId(result.photo_id ?? photos.photos.find(p => p.item_id === id)?.id ?? null);
      if (result.goal) setGoalText([String(result.goal.x_m), String(result.goal.y_m), String(result.goal.yaw_rad * 180 / Math.PI)]);
    } catch (failure) { if (!controller.signal.aborted) setError((failure as Error).message); }
    finally { if (resolutionRequest.current === controller) { resolutionRequest.current = null; setBusy(false); } }
  }
  useEffect(() => {
    if (photoMode) setMaximized(current => current && current !== 'operate' ? 'operate' : current);
  }, [photoMode]);

  async function captureGraspGoal(itemId: string, revision: number, step: 'height' | 'goal') {
    if (!packet || !zones) throw new Error('Current map unavailable');
    await controlReady();
    return api<PhotoIndex>(`/api/items/grasp-${step}`, { method: 'POST', body: JSON.stringify({
      id: commandId(), item_id: itemId, session_id: session.current, stop_epoch: state?.hardware?.stop_epoch,
      map_id: packet.info.map_id, expected_revision: revision, zone_revision: zones.revision,
    }) });
  }

  function photoChanged(index: PhotoIndex, addedId?: string) {
    setPhotos(index);
    const id = addedId ?? selectedPhotoId;
    setSelectedPhotoId(id);
    setSelectedItemId(index.photos.find(p => p.id === id)?.item_id ?? '');
    if (addedId) setPhotoMode(index.photos.find(p => p.id === addedId)?.kind === 'reference' ? 'uploaded' : 'edit');
  }
  async function capture(camera_id: 'front' | 'arm') {
    if (!canEdit || !packet || !photos || busy) return;
    setBusy(true); setError('');
    try {
      await controlReady();
      const result = await api<PhotoIndex & { added_id: string }>('/api/photos/capture', { method: 'POST',
        body: JSON.stringify({ id: commandId(), session_id: session.current, stop_epoch: state?.hardware?.stop_epoch, camera_id, map_id: packet.info.map_id, expected_revision: photos.revision, zone_revision: zones?.revision }) });
      photoChanged(result, result.added_id);
    } catch (failure) { setError((failure as Error).message); await reload().catch(() => {}); }
    finally { setBusy(false); }
  }
  function storedCamera(camera: 'front' | 'arm') {
    const live = state?.cameras?.[camera];
    return live ?
      <div className="stored-photo"><img src={live.url} alt={`Live ${camera} camera`} /></div> :
      <div className="camera-body"><Camera size={26} strokeWidth={1} /><span>Camera unavailable</span></div>;
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
          <Dropdown.Item className="menu-item" disabled={!canEdit || busy || !photos} onSelect={() => setPhotoMode('import')}><Camera size={16} /> Import reference photo</Dropdown.Item>
          <Dropdown.Item className="menu-item" disabled={!canLayout} onSelect={() => setLayoutEditing(v => !v)}><LayoutGrid size={16} /> {layoutEditing ? 'Lock layout' : 'Edit layout'}</Dropdown.Item>
          <Dropdown.Item className="menu-item" disabled={!canLayout} onSelect={resetLayout}><RotateCcw size={16} /> Restore default layout</Dropdown.Item>
        </Dropdown.Content></Dropdown.Portal>
      </Dropdown.Root>
      <strong className="brand">MYAGV CONTROL</strong><span className="demo-tag">{state?.demo === false ? 'HARDWARE' : 'DEMO'}</span>
      <span className={`connection ${connected ? 'online' : ''}`}>{connected ? 'Connected' : 'Offline'}</span>
      <BatteryStatus />
      <Button variant="danger" onClick={() => void stop()} className="stop">STOP</Button>
    </header>
    {error && <div role="alert" className="error-bar"><span>{error}</span></div>}
    <div className="workspace" ref={workspaceHost}>
    <main className={`grid-stack ${layoutEditing ? 'layout-editing' : ''}`} ref={gridHost}>
      {panel('map', <>
        <div className="map-tools">
          {state?.demo === false && <label className="muted obstacle-filter"><span>Filter small obstacles · {Math.round((obstacleSize ?? state.hardware?.small_obstacle_m ?? .15)*100)} cm</span>
            <input aria-label="Filter small obstacles" type="range" min="0" max="0.5" step="0.01"
              disabled={!canEdit || busy || state.hardware?.phase !== 'idle'} value={obstacleSize ?? state.hardware?.small_obstacle_m ?? .15}
              onChange={e => setObstacleSize(Number(e.target.value))}
              onPointerUp={e => { void hardwareCommand('obstacle_filter', { minimum_m: Number(e.currentTarget.value) }); }}
              onKeyUp={e => { if (['ArrowLeft', 'ArrowRight', 'Home', 'End'].includes(e.key)) void hardwareCommand('obstacle_filter', { minimum_m: Number(e.currentTarget.value) }); }} />
          </label>}
          {(drawing || !state?.hardware?.mapping_mode) && <span className="muted">{drawing ? draft.length < 2 ? 'Select two corners' : 'Preview' : state?.demo === false ? 'Saved ROS map' : 'Demo map'}</span>}
          <div className="toolbar-actions">
            {state?.demo === false && <Button disabled={!canEdit || busy} onClick={() => setResetMapOpen(true)}><RotateCcw size={14} /> Reset map</Button>}
            {drawing && <><Button variant="default" disabled={draft.length !== 2 || busy || !canEdit} onClick={() => void command('zone_add')}>Save</Button>
              <Button onClick={() => { setDrawing(false); setDraft([]); }}>Cancel</Button></>}
            {selected && !drawing && <Button disabled={!canEdit || busy} onClick={() => void command('zone_delete')}><Trash2 size={14} /> Delete</Button>}
            <Button variant="ghost" className="icon" aria-label="Fit map" onClick={() => setFitVersion(v => v + 1)}><Crosshair size={16} /></Button>
            <Dropdown.Root><Dropdown.Trigger asChild><Button variant="ghost" className="icon" aria-label="Map actions"><MoreHorizontal size={18} /></Button></Dropdown.Trigger>
              <Dropdown.Portal><Dropdown.Content className="menu" align="end" sideOffset={5}>
                <Dropdown.Item className="menu-item" disabled={!canEdit || !packet || busy} onSelect={startDrawing}><Square size={15} /> No-go zone</Dropdown.Item>
                <Dropdown.Item className="menu-item" disabled={!navigation} onSelect={() => setFocusRobotVersion(v => v + 1)}><Crosshair size={15} /> Fit robot</Dropdown.Item>
                <Dropdown.Item className="menu-item" disabled={!canEditPhotos || busy || !photos?.items.length || state?.llm?.status !== 'configured'} onSelect={() => void estimatePositions()}><Search size={15} /> Estimate item positions</Dropdown.Item>
                <Dropdown.CheckboxItem className="menu-item" disabled={!costmapReady} checked={showCostmap} onCheckedChange={setShowCostmap}><Square size={15} /> Global costmap {showCostmap ? '✓' : ''}</Dropdown.CheckboxItem>
                <Dropdown.Item className="menu-item" disabled={!state?.demo || !canEdit || busy} onSelect={() => void simulationCommand('demo_reset')}><RotateCcw size={15} /> Reset demo pose</Dropdown.Item>
                <Dropdown.Item className="menu-item" onSelect={() => setSettingsOpen(true)}><Settings2 size={15} /> Settings</Dropdown.Item>
              </Dropdown.Content></Dropdown.Portal>
            </Dropdown.Root>
          </div>
        </div>
        {packet ? <MapView info={packet.info} image={packet.image} zones={zones?.zones ?? []} selected={selected}
          drawing={drawing && canEdit} draft={draft} fitVersion={fitVersion} onDraft={setDraft} onSelect={setSelected}
          navigation={navigation} goal={goal} connected={connected} showCostmap={showCostmap} focusRobotVersion={focusRobotVersion}
          photos={photos?.photos ?? []} selectedPhotoId={selectedPhotoId} onPhoto={selectPhoto} update={state?.hardware?.map_update}
          locations={photos?.locations ?? []} onLocation={location => void selectLocation(location)}
          onGoal={previewGoal} /> : <div className="empty">Loading map…</div>}
        {locationPhotos.some(p => p.location_analysis?.status === 'running' || p.location_analysis?.status === 'queued') &&
          <span className="muted location-status" role="status">Estimating item positions…</span>}
        {locationPhotos.some(p => p.location_analysis?.status === 'error') &&
          <span className="muted location-status" role="status">Position estimation failed; retry in Map actions.</span>}
        {state?.demo === false && <MapUpdateControls state={state.hardware} connected={connected} goal={goal} command={hardwareCommand} />}
        {drawing && <details className="coordinate-editor"><summary>Coordinates · view meters</summary>
          <div className="coordinate-row">{['U1', 'V1', 'U2', 'V2'].map((label, i) => <label key={label}>{label}<input type="number" step="0.05"
            value={keyboardCorners[i]} onChange={e => setKeyboardCorners(v => v.map((item, n) => n === i ? e.target.value : item))} /></label>)}
            <Button disabled={!keyboardCorners.every(v => v.trim() !== '' && Number.isFinite(Number(v)))} onClick={() => {
              const [u1, v1, u2, v2] = keyboardCorners.map(Number); setDraft([[v1, u1], [v2, u2]]);
            }}>Preview</Button>
          </div>
        </details>}
      </>)}
      {panel('front', storedCamera('front'), <Button variant="ghost" className="icon" aria-label="Capture front observation" title="Capture front observation" disabled={!canEdit || busy || !photos || (!state?.demo && !state?.hardware?.localized)} onClick={() => void capture('front')}><Camera size={15} /></Button>)}
      {panel('arm', storedCamera('arm'), <Button variant="ghost" className="icon" aria-label="Capture arm observation" title="Capture arm observation" disabled={!canEdit || busy || !photos || (!state?.demo && !state?.hardware?.localized)} onClick={() => void capture('arm')}><Camera size={15} /></Button>)}
      {panel('operate', <div className="operate-pages"><div className="operate-body" inert={Boolean(photoMode)} aria-hidden={Boolean(photoMode)}>
        <label className="subheading" htmlFor="item-request">REQUEST</label>
        <form className="request-row" onSubmit={event => { event.preventDefault(); void resolveRequest(); }}>
          <input id="item-request" value={requestText} maxLength={500} placeholder="Bring me the red cup" disabled={!canEdit || busy}
            onChange={event => updateRequest(event.target.value)} />
          <Button type="submit" variant="default" className="icon" aria-label={resolutionRequest.current ? 'Resolving…' : 'Resolve'} title="Resolve" aria-busy={Boolean(resolutionRequest.current)}
            disabled={!canEdit || busy || !photos || !requestText.trim() || state?.llm?.status !== 'configured' || settingsDirty || settingsSaving || Boolean(saveError)}>{resolutionRequest.current ? <LoaderCircle size={16} /> : <Search size={16} />}</Button>
          <Button type="button" variant="ghost" className="icon" aria-label="Random Japanese request" title="Random Japanese request" disabled={!canEdit || busy}
            onClick={() => updateRequest(REQUEST_EXAMPLES[Math.floor(Math.random() * REQUEST_EXAMPLES.length)])}><Dice5 size={16} /></Button>
        </form>
        {state?.llm?.status !== 'configured' && <span className="muted">LLM {state?.llm?.status === 'unavailable' ? 'unavailable' : 'not configured'}</span>}
        {resolution && <div role="status" className="resolution-result">
          <span className="muted">{resolution.status === 'not_found' ? 'No matching item' : resolution.status === 'ambiguous' ? 'Choose item' : resolution.blocked_reason ?? (resolution.target_source === 'estimated' ? 'Estimated approach selected; use Go' : 'Target selected')}</span>
          {resolution.status === 'ambiguous' && <div className="candidate-list">{resolution.item_ids.map(id => <Button key={id} disabled={!canEdit || busy} onClick={() => selectItem(id)}>{photos?.items.find(i => i.id === id)?.name}</Button>)}</div>}
        </div>}
        <label className="subheading" htmlFor="item-select">ITEM</label>
        <div className="request-row">
          <select id="item-select" value={selectedItemId} disabled={busy} onChange={e => selectItem(e.target.value)}><option value="">All photos</option>{photos?.items.map(i => <option value={i.id} key={i.id}>{i.name}</option>)}</select>
          <Button variant="ghost" className="icon" style={{ color: 'var(--orange-11)' }} aria-label="Import reference photo" title="Import reference photo"
            disabled={!canEdit || busy || !photos} onClick={() => setPhotoMode('import')}><Plus size={18} /></Button>
        </div>
        <div className="photo-list">{photos?.photos.length ? [...photos.photos].reverse().filter(p => !selectedItemId || p.item_id === selectedItemId).map(p =>
          <button key={p.id} className={`zone-row ${p.id === selectedPhotoId ? 'selected' : ''}`} onClick={() => selectPhoto(p.id)} title={p.kind === 'reference' ? 'Phone reference · no map point' : 'Captured base observation pose'}>
            <Camera size={14} /><span>{photos.items.find(i => i.id === p.item_id)?.name ?? 'Unlabelled'}</span><span className="row-end">{p.kind === 'reference' ? 'Reference' : p.current === false ? 'History' : p.camera_id}</span>
          </button>) : <span className="muted">No photos</span>}</div>
        {selectedItem && selectedPhoto?.item_id === selectedItem.id && <div className="item-photo"><StoredPhoto key={selectedPhoto.id} photo={selectedPhoto} onEdit={() => setPhotoMode('edit')} /></div>}
        {selectedPhoto && <div className="action-line"><Button onClick={() => setPhotoMode('edit')}>Edit</Button>
          {selectedPhoto.kind === 'observation' && <Button disabled={!canEdit || !selectedPhoto.map_matches || !selectedPhoto.available} onClick={() => {
            const pose = selectedPhoto.base_pose; setGoalText([String(pose.x_m), String(pose.y_m), String(pose.yaw_rad * 180 / Math.PI)]);
          }}>View point</Button>}
          {selectedItem?.station_status === 'ready' && <Button disabled={!canEdit} onClick={() => {
            const base = stations[selectedItem.station_link!.name]?.base;
            if (base) setGoalText([String(base.x_m), String(base.y_m), String(base.yaw_deg)]);
          }}>Approach</Button>}
        </div>}
        <span className="subheading">{state?.demo === false ? 'POSE · ROS ESTIMATE' : 'POSE · DEMO'}</span>
        <output className="pose-readout" aria-label={state?.demo === false ? 'ROS map-frame pose; nominal chassis outline' : 'Simulated map-frame pose'}>
          <span>X <b>{navigation?.pose?.x_m.toFixed(2) ?? '—'}</b> m</span>
          <span>Y <b>{navigation?.pose?.y_m.toFixed(2) ?? '—'}</b> m</span>
          <span>θ <b>{navigation?.pose ? (navigation.pose.yaw_rad * 180 / Math.PI).toFixed(0) : '—'}</b>°</span>
        </output>
        <span className="subheading">GOAL</span>
        <div className="goal-inputs">{['X (m)', 'Y (m)', 'θ (°)'].map((label, i) => <label key={label}>{label}<input type="number" step={i === 2 ? '5' : '.05'}
          min={i === 2 ? -180 : undefined} max={i === 2 ? 180 : undefined} placeholder={i === 2 ? '0' : 'Map click'} value={goalText[i]}
          disabled={!canLayout || busy} onChange={e => setGoalText(old => old.map((v, n) => n === i ? e.target.value : v))} /></label>)}</div>
        <div className="action-line">{state?.demo ? <Button variant="default" className="button-simulate" disabled={!canEdit || !goalValid || !costmapReady || busy}
          onClick={() => void simulationCommand('navigate')}><Play size={16} /> Simulate</Button> : <>
          <Button variant="default" disabled={goReasons.length > 0} title={goReasons.join('; ')} onClick={() => void simulationCommand('navigate')}><Crosshair size={16} /> Go</Button>
          <Button variant="default" disabled={fetchReasons.length > 0} title={fetchReasons.join('; ')}
            onClick={() => void hardwareCommand('fetch', { item_id: selectedItemId, index_revision: photos?.revision })}><Hand size={16} /> Fetch</Button>
          </>}
          <Button disabled={!connected || state?.phase === 'idle'} onClick={() => void simulationCommand('cancel')}>Cancel</Button></div>
        {state?.demo === false && <div className="muted" role="status">
          {goReasons.length > 0 && <p>Go needs: {goReasons.join('; ')}.</p>}
          {fetchReasons.length > 0 && <p>Fetch needs: {fetchReasons.join('; ')}.</p>}
        </div>}
        <div className="section-divider" />
        <span className="subheading">NO-GO ZONES</span>
        <div className="zone-list">{zones?.zones.length ? zones.zones.map((zone, i) => <button key={zone.id}
          className={`zone-row ${selected === zone.id ? 'selected' : ''}`} onClick={() => { setSelected(zone.id); setDrawing(false); setDraft([]); }}>
          <Square size={14} /> Zone {i + 1}<span className="row-end">{selected === zone.id ? 'Selected' : ''}</span>
        </button>) : <span className="muted">None</span>}</div>
        <span className="muted enforcement">{state?.demo === false ? costmapReady ? 'ROS global + local no-go applied' : 'ROS no-go enforcement pending' : costmapReady ? 'Demo costmap applied' : 'Costmap pending'}</span>
        {state?.demo === false && <HardwareControls stations={stations} state={state.hardware} connected={connected} goal={goal} command={hardwareCommand} hold={input => { held.current = input; }} stop={() => void stop()} />}
        <div className="section-divider" /><span className="subheading">TASK</span><span className="muted" role="status">{connected ? navigation?.status ?? 'Loading…' : 'Offline'}</span>
        <FetchTest connected={connected} editable={canEdit && !busy} itemId={selectedItemId} itemName={selectedItem?.name}
          indexRevision={photos?.revision} command={hardwareCommand} stop={() => void stop()}
          reasons={[
            ...(!canEdit || busy ? ['Connect and stop the robot'] : []),
            ...(state?.demo !== false ? ['Live hardware required'] : []),
            ...(!state?.hardware?.arm_homed || !state.hardware.localized ? ['Home arm and confirm localization'] : []),
            ...(state?.llm?.status !== 'configured' ? ['LLM not configured'] : []),
            ...(!selectedItem?.grasp_goal_ready ? ['Select an item with saved grasp GOAL views'] : []),
          ]} />
      </div>{photoMode && photos && <PhotoEditor key={photoMode === 'import' ? 'import' : selectedPhotoId}
        photo={photoMode !== 'import' ? selectedPhoto : null} autoDescribe={photoMode === 'uploaded'}
        index={photos} stations={stations} enabled={canEditPhotos}
        goalCaptureReason={!connected ? 'Reconnect to the robot.'
          : state?.phase === 'teleop' ? 'Exit TELEOP, then verify stopped hardware and confirm Already homed in OPERATE. Saved grasp Z is retained.'
          : state?.phase === 'fault' ? 'In OPERATE, verify stopped hardware and clear the fault, then confirm Already homed.'
          : state?.phase !== 'idle' ? 'Finish or stop the current action before recording.'
          : !state?.hardware?.arm_available ? 'P340 feedback unavailable; reconnect the arm.'
          : !state.hardware.arm_homed ? 'In OPERATE, confirm Already homed if the arm has homed since power-on.'
          : !state.hardware.localized ? 'Confirm the robot map pose in OPERATE.' : ''}
        onCaptureGoal={captureGraspGoal} onClose={() => setPhotoMode(null)} onChange={photoChanged} onBusy={setBusy} onStop={() => void stop()} />}</div>)}
    </main>
    <div className="panel-dividers">
      {!maximized && dividers(layout).map(seam => {
        const vertical = seam.axis === 'x';
        return <button key={`${seam.axis}:${seam.before.join(',')}:${seam.after.join(',')}`}
          className={`panel-divider ${vertical ? 'vertical' : 'horizontal'}`} role="separator"
          aria-orientation={vertical ? 'vertical' : 'horizontal'}
          aria-label={`Resize ${seam.before.map(id => names[id]).join(' / ')} and ${seam.after.map(id => names[id]).join(' / ')}`}
          aria-valuenow={seam.position} aria-valuemin={0} aria-valuemax={vertical ? gridSize.columns : Math.max(...layout.map(t => t.y + t.h))}
          disabled={!canResize} title={canResize ? 'Drag to resize adjacent panels' : 'Resize unavailable while offline or navigating'}
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
    <Dialog.Root open={resetMapOpen} onOpenChange={open => { if (!busy) setResetMapOpen(open); }}>
      <Dialog.Portal><Dialog.Overlay className="dialog-overlay" /><Dialog.Content className="settings-dialog">
        <header className="dialog-header"><Dialog.Title>Reset map and rebuild?</Dialog.Title></header>
        <div className="settings-body"><Dialog.Description>
          Clear the current map, no-go zones and taught stations, then start live SLAM from fresh scans.
          Old map data is archived for recovery. Photos stay, but old map positions and station links become invalid.
          Keep the robot stopped during reset. Verify the new map pose before driving.
        </Dialog.Description>{error && <p role="alert">{error}</p>}</div>
        <footer className="dialog-footer"><Button disabled={busy} onClick={() => setResetMapOpen(false)}>Cancel</Button>
          <Button variant="danger" disabled={!canEdit || busy} onClick={async () => {
            setBusy(true);
            try { await hardwareCommand('reset_map', { confirmed: true }); setResetMapOpen(false); }
            catch { /* Keep the confirmation open with the reported error. */ }
            finally { setBusy(false); }
          }}>{busy ? 'Resetting…' : 'Reset and start live mapping'}</Button></footer>
      </Dialog.Content></Dialog.Portal>
    </Dialog.Root>
    <Dialog.Root open={settingsOpen} onOpenChange={setSettingsOpen}>
      <Dialog.Portal><Dialog.Overlay className="dialog-overlay" /><Dialog.Content className="settings-dialog">
        <header className="dialog-header"><Dialog.Title>SETTINGS</Dialog.Title><Dialog.Close asChild><Button variant="ghost" className="icon" aria-label="Close settings"><X size={18} /></Button></Dialog.Close></header>
        <Dialog.Description className="sr-only">Map display and workspace settings</Dialog.Description>
        <div className="settings-body">
          <span className="subheading">LLM</span><span className="muted">OpenRouter · {state?.llm?.status ?? 'Loading…'}</span>
          <label className="setting-row">Model<select value={settingsDraft.llm_model} disabled={!canEdit || !savedSettings || busy} onChange={event => {
            const model = savedSettings?.llm_models.find(m => m.id === event.target.value);
            if (model) updateSetting({ ...settingsDraft, llm_model: model.id, reasoning_effort: model.efforts.includes(settingsDraft.reasoning_effort) ? settingsDraft.reasoning_effort : model.efforts[0] });
          }}>{savedSettings?.llm_models.map(model => <option value={model.id} key={model.id}>{model.name}</option>)}</select></label>
          <label className="setting-row">Thinking<select value={settingsDraft.reasoning_effort} disabled={!canEdit || !savedSettings || busy}
            onChange={event => updateSetting({ ...settingsDraft, reasoning_effort: event.target.value })}>
            {savedSettings?.llm_models.find(model => model.id === settingsDraft.llm_model)?.efforts.map(effort => <option value={effort} key={effort}>{effort === 'off' ? 'Off' : effort[0].toUpperCase() + effort.slice(1)}</option>)}
          </select></label><div className="section-divider" />
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
        <footer className="dialog-footer"><span className="muted">{canEdit ? 'Auto-save' : connected ? 'Stop navigation to edit' : 'Offline'}</span><Button variant="danger" onClick={() => void stop()}>STOP</Button></footer>
      </Dialog.Content></Dialog.Portal>
    </Dialog.Root>
  </div>;
}
