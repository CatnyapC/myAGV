const KEY = 'myagv:photo-selection:v1';
type Selection = { itemId: string; photoId: string | null };

export function loadPhotoSelection(): Selection {
  try {
    const value = JSON.parse(localStorage.getItem(KEY) ?? 'null');
    if (value && typeof value.itemId === 'string' && (value.photoId === null || typeof value.photoId === 'string')) return value;
  } catch { /* Storage can be unavailable. */ }
  return { itemId: '', photoId: null };
}

export function savePhotoSelection(itemId: string, photoId: string | null): void {
  try { localStorage.setItem(KEY, JSON.stringify({ itemId, photoId })); }
  catch { /* Keep selection usable when storage is unavailable. */ }
}
