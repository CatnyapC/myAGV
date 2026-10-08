import { useEffect, useState } from 'react';
import { ArrowLeft, Camera } from 'lucide-react';
import { api } from './api';
import { Button } from './ui';
import type { Photo, PhotoIndex, Stations } from './mapGeometry';

export function StoredPhoto({ photo, onEdit }: { photo: Photo; onEdit: () => void }) {
  const [failed, setFailed] = useState(false);
  return <div className="stored-photo">
    {photo.available && !failed ? <img src={photo.image_url} alt={photo.kind === 'reference' ? 'Item reference photo' : 'Map observation photo'} onError={() => setFailed(true)} /> : <span className="empty">Image unavailable</span>}
    <button className="photo-caption" onClick={onEdit}>{photo.kind === 'reference' ? 'Reference · phone' : photo.source === 'ros' ? 'Observation · ROS' : 'Observation · Demo'} · Edit</button>
  </div>;
}

async function normalizedPng(file: File): Promise<Blob> {
  if (!['image/png', 'image/jpeg', 'image/webp'].includes(file.type) || file.size > 16 * 1024 * 1024) throw new Error('Choose PNG/JPEG/WebP up to 16 MiB');
  const url = URL.createObjectURL(file);
  try {
    const image = new Image(); image.src = url; await image.decode();
    const scale = Math.min(1, 1600 / Math.max(image.width, image.height));
    const canvas = document.createElement('canvas');
    canvas.width = Math.max(1, Math.round(image.width * scale)); canvas.height = Math.max(1, Math.round(image.height * scale));
    const context = canvas.getContext('2d');
    if (!context) throw new Error('Image conversion unavailable');
    context.drawImage(image, 0, 0, canvas.width, canvas.height);
    const blob = await new Promise<Blob | null>(resolve => canvas.toBlob(resolve, 'image/png'));
    if (!blob || blob.size > 4 * 1024 * 1024) throw new Error('Converted image exceeds 4 MiB');
    return blob;
  } finally { URL.revokeObjectURL(url); }
}

export function PhotoEditor(props: {
  photo: Photo | null; index: PhotoIndex; stations: Stations; enabled: boolean; autoDescribe: boolean;
  goalCaptureEnabled: boolean; onCaptureGoal: (itemId: string, revision: number, step: 'height' | 'goal') => Promise<PhotoIndex>;
  onClose: () => void; onChange: (index: PhotoIndex, addedId?: string) => void; onBusy: (value: boolean) => void; onStop: () => void;
}) {
  const currentItem = props.index.items.find(i => i.id === props.photo?.item_id);
  const [revision, setRevision] = useState(props.index.revision);
  const [file, setFile] = useState<File | null>(null);
  const [itemId, setItemId] = useState(currentItem?.id ?? (props.photo?.kind === 'reference' ? 'new' : ''));
  const [name, setName] = useState(currentItem?.name ?? '');
  const [appearance, setAppearance] = useState(currentItem?.appearance ?? '');
  const [station, setStation] = useState(currentItem?.station_link?.name ?? '');
  const [stationConfirmed, setStationConfirmed] = useState(false);
  const [error, setError] = useState('');
  const [busy, setBusy] = useState(false);
  const [describeRequested, setDescribeRequested] = useState<{ name?: string } | null>(props.autoDescribe ? {} : null);
  useEffect(() => { if (!props.enabled) setDescribeRequested(null); }, [props.enabled]);
  useEffect(() => {
    if (!describeRequested || !props.photo || !props.enabled) return;
    const controller = new AbortController();
    setBusy(true); props.onBusy(true); setError('');
    void api<{ name?: string; appearance: string }>(`/api/photos/${props.photo.id}/describe`, {
      method: 'POST', signal: controller.signal, body: JSON.stringify({ expected_revision: revision, ...describeRequested }),
    }).then(result => {
      if (controller.signal.aborted) return;
      setItemId(id => id || 'new');
      if (!('name' in describeRequested)) setName(value => value.trim() ? value : result.name ?? value);
      setAppearance(result.appearance);
    }).catch(failure => {
      if (!controller.signal.aborted) setError((failure as Error).message);
    }).finally(() => {
      if (!controller.signal.aborted) { setDescribeRequested(null); setBusy(false); props.onBusy(false); }
    });
    return () => { controller.abort(); setBusy(false); props.onBusy(false); };
  }, [describeRequested, props.photo?.id, props.enabled]);
  async function mutate(remove = false, record?: 'height' | 'goal') {
    if (!props.enabled || busy) return;
    setBusy(true); props.onBusy(true); setError('');
    try {
      if (!props.photo) {
        if (!file) return;
        const blob = await normalizedPng(file);
        const result = await api<PhotoIndex & { added_id: string }>(`/api/photos/reference?expected_revision=${revision}`,
          { method: 'POST', headers: { 'Content-Type': 'image/png' }, body: blob });
        props.onChange(result, result.added_id);
      } else {
        const result = await api<PhotoIndex>(`/api/photos/${props.photo.id}`, { method: remove ? 'DELETE' : 'PATCH',
          body: JSON.stringify({ expected_revision: revision,
            ...(!remove ? { item_id: itemId, name, appearance, station_name: station, confirmed: true, station_confirmed: stationConfirmed } : {}) }) });
        props.onChange(result); setRevision(result.revision);
        if (record) {
          const savedItemId = result.photos.find(p => p.id === props.photo?.id)?.item_id;
          if (!savedItemId) throw new Error('Save an item before recording its grasp goal');
          setItemId(savedItemId);
          const captured = await props.onCaptureGoal(savedItemId, result.revision, record);
          props.onChange(captured); setRevision(captured.revision);
        } else props.onClose();
      }
    } catch (failure) { setError((failure as Error).message); }
    finally { setBusy(false); props.onBusy(false); }
  }
  const photo = props.photo;
  const editingItem = props.index.items.find(i => i.id === itemId);
  const stationNeedsConfirmation = Boolean(station) && (editingItem?.station_link?.name !== station || editingItem?.station_status !== 'ready');
  return <section className="photo-page" aria-label={photo ? 'Edit item' : 'Import item photo'}>
      <header className="photo-page-header">
        <Button variant="ghost" className="icon" aria-label="Back to OPERATE" autoFocus disabled={busy} onClick={props.onClose}><ArrowLeft size={18} /></Button>
        <h2>{photo ? 'ITEM' : 'IMPORT PHOTO'}</h2>
      </header>
      <div className="settings-body">
        {!photo ? <><label className="setting-row">Phone photo<input type="file" accept="image/png,image/jpeg,image/webp" disabled={!props.enabled || busy} onChange={e => setFile(e.target.files?.[0] ?? null)} /></label>
          <span className="muted">LLM generates name and appearance after import · no map point</span></> : <>
          {photo.available && <img className="photo-preview" src={photo.image_url} alt="Stored photo" />}
          {photo.kind === 'observation' && <span className="muted">{photo.source === 'simulation' ? 'Synthetic observation · simulated capture pose' : photo.current === false ? 'Historical observation · view has since changed' : 'Last observed · measured base capture pose'}</span>}
          <Button disabled={!props.enabled || busy || !photo.available} onClick={() => setDescribeRequested({ name })}>
            {describeRequested && busy ? 'Generating description…' : 'Generate with LLM'}</Button>
          {photo.kind === 'observation' && <details><summary className="muted">Capture metadata</summary><div className="photo-metadata">
            <span>{new Date(photo.captured_at_s * 1000).toLocaleString()} · {photo.camera_id}</span>
            <span>X {photo.base_pose.x_m.toFixed(2)} m · Y {photo.base_pose.y_m.toFixed(2)} m · θ {(photo.base_pose.yaw_rad * 180 / Math.PI).toFixed(0)}°</span>
            <span>{photo.map_matches ? 'Current map' : 'Map mismatch'} · base observation pose</span>
          </div></details>}
          <label className="setting-row">Item<select value={itemId} disabled={!props.enabled || busy} onChange={e => {
            const id = e.target.value, item = props.index.items.find(i => i.id === id);
            setItemId(id); setName(item?.name ?? ''); setAppearance(item?.appearance ?? ''); setStation(item?.station_link?.name ?? ''); setStationConfirmed(false);
          }}><option value="">Unlabelled</option><option value="new">New item</option>{props.index.items.map(i => <option key={i.id} value={i.id}>{i.name}</option>)}</select></label>
          {itemId && <><label className="photo-field">Name<input value={name} maxLength={100} disabled={!props.enabled || busy} onChange={e => setName(e.target.value)} /></label>
            <label className="photo-field">Appearance<textarea value={appearance} maxLength={1000} rows={3} disabled={!props.enabled || busy} placeholder="Color, shape, distinguishing features" onChange={e => setAppearance(e.target.value)} /></label>
            <label className="setting-row">Station<select value={station} disabled={!props.enabled || busy} onChange={e => { setStation(e.target.value); setStationConfirmed(false); }}><option value="">Unlinked</option>
              {station && !props.stations[station] && <option value={station}>Unavailable: {station}</option>}
              {Object.keys(props.stations).map(n => <option key={n} value={n}>{n}</option>)}</select></label>
            {stationNeedsConfirmation && props.stations[station] && <><span className="muted">X {props.stations[station].base.x_m.toFixed(2)} m · Y {props.stations[station].base.y_m.toFixed(2)} m · θ {props.stations[station].base.yaw_deg.toFixed(0)}°</span>
              <label className="setting-row">Confirm station<input type="checkbox" checked={stationConfirmed} disabled={!props.enabled || busy} onChange={e => setStationConfirmed(e.target.checked)} /></label></>}
            {editingItem && !['ready', 'none'].includes(editingItem.station_status) && <span className="muted">Station link {editingItem.station_status}</span>}
            <div className="section-divider" /><span className="subheading">1. GRASP HEIGHT</span>
            <span className="muted">Manually grasp this item, then record the current arm Z coordinate.</span>
            <Button disabled={!props.enabled || !props.goalCaptureEnabled || busy || !name.trim() || (stationNeedsConfirmation && !stationConfirmed)}
              onClick={() => void mutate(false, 'height')}>Save current grasp Z</Button>
            <span className="muted">{editingItem?.grasp_z_mm != null ? `Grasp Z saved: ${editingItem.grasp_z_mm.toFixed(1)} mm` : 'Grasp height not recorded yet.'}</span>
            <span className="muted">Next, return to OPERATE controls, release the item in place, and raise the arm to the alignment position above the saved grasp height so adjustments will not knock over the item. Then return here to save both camera views.</span>
            <span className="subheading">2. ALIGNMENT CAMERA GOAL</span>
            <Button disabled={!props.enabled || !props.goalCaptureEnabled || busy || !name.trim() || editingItem?.grasp_z_mm == null || (stationNeedsConfirmation && !stationConfirmed)}
              onClick={() => void mutate(false, 'goal')}><Camera size={15} />Save item & capture both cameras</Button>
            <span className="muted">{editingItem?.grasp_goal_ready ? 'Alignment views saved.' : editingItem?.grasp_goal ? 'Saved views unavailable; capture both cameras again.' : 'No alignment goal saved yet.'}</span>
            {editingItem?.grasp_goal && <div className="grasp-goal-views">{(['front', 'arm'] as const).map(camera => {
              const goalPhoto = props.index.photos.find(p => p.id === editingItem.grasp_goal?.[camera]);
              return goalPhoto?.available && <figure key={camera}><img src={goalPhoto.image_url} alt={`${camera} correct grasp goal`} /><figcaption>{camera.toUpperCase()} GOAL</figcaption></figure>;
            })}</div>}
          </>}
        </>}
        {error && <span role="alert" className="muted">{error}</span>}
      </div><footer className="dialog-footer">
        {photo ? <Button disabled={!props.enabled || busy} onClick={() => void mutate(true)}>Delete photo</Button> : <span />}
        <Button variant="danger" onClick={props.onStop}>STOP</Button>
        <Button variant="default" disabled={!props.enabled || busy || (!photo && !file) || (Boolean(photo) && Boolean(itemId) && (!name.trim() || (stationNeedsConfirmation && !stationConfirmed)))}
          onClick={() => void mutate()}>{busy ? describeRequested ? 'Generating…' : 'Saving…' : !photo ? 'Import reference' : photo.kind === 'observation' && itemId ? 'Confirm association' : 'Save'}</Button>
      </footer>
    </section>;
}
