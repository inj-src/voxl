// The remote cursor, drawn locally by the browser so pointer movement has no stream
// latency. Cursor messages:
// - flags=0x00: type(0x03), flags(0), w, h, hx, hy (u16 each), id (u32), raw RGBA
// - flags=0x01: type(0x03), flags(1), w, h, hx, hy (u16 each), id (u32), PNG bytes
// - flags=0x02: type(0x03), flags(2), id (u32) — cache hit
import type { App } from './app';
import { el } from './app';

// On-screen cursor size bounds in CSS pixels, applied to the image's longer side.
// 24 is the standard pointer size (Xcursor's default), so a desktop scaled down
// into a small window (an embedded sidebar) keeps a normal-sized pointer.
const MIN_CURSOR_PX = 24;
const MAX_CURSOR_PX = 128;
const MAX_CURSOR_CACHE = 64;

interface CachedCursor {
	w: number;
	h: number;
	hx: number;
	hy: number;
	source: HTMLCanvasElement | ImageBitmap;
}

export function createCursor(app: App) {
	let current: CachedCursor | null = null;
	let activeId = 0;
	const cache = new Map<number, CachedCursor>();
	const scaled = document.createElement('canvas');
	/** Trackpad mode's cursor (pointer.ts), which shows the same image. */
	const pointer = el('pointer');

	function remember(id: number, entry: CachedCursor) {
		if (!id) return;
		const old = cache.get(id);
		if (old && old.source instanceof ImageBitmap) old.source.close();
		cache.delete(id);
		cache.set(id, entry);
		while (cache.size > MAX_CURSOR_CACHE) {
			const firstKey = cache.keys().next().value;
			if (firstKey === undefined) break;
			const evicted = cache.get(firstKey);
			cache.delete(firstKey);
			if (evicted && evicted !== current && evicted.source instanceof ImageBitmap)
				evicted.source.close();
		}
	}

	function apply() {
		if (!current) return;
		const { w, h, hx, hy, source } = current;
		if (!w || !h) {
			app.surface.style.cursor = 'none';
			pointer.classList.add('none');
			return;
		}
		// Cursor images are in desktop pixels; show them at the on-screen scale, but
		// never smaller than a normal pointer.
		const dpr = window.devicePixelRatio || 1;
		const size = Math.max(w, h);
		const s = Math.min(
			Math.max(app.view.scale, MIN_CURSOR_PX / size),
			MAX_CURSOR_PX / size
		);
		const cw = Math.max(1, Math.round(w * s)),
			ch = Math.max(1, Math.round(h * s));
		const dpw = Math.max(1, Math.round(cw * dpr)),
			dph = Math.max(1, Math.round(ch * dpr));
		scaled.width = dpw;
		scaled.height = dph;
		const cx = scaled.getContext('2d')!;
		cx.imageSmoothingQuality = 'high';
		cx.clearRect(0, 0, dpw, dph);
		cx.drawImage(source, 0, 0, dpw, dph);
		const url = scaled.toDataURL('image/png');
		const hsx = Math.round(hx * s),
			hsy = Math.round(hy * s);
		app.surface.style.cursor =
			dpr > 1
				? `-webkit-image-set(url(${url}) ${dpr}x) ${hsx} ${hsy}, url(${url}) ${hsx} ${hsy}, default`
				: `url(${url}) ${hsx} ${hsy}, default`;
		if (dpr > 1 && !app.surface.style.cursor)
			app.surface.style.cursor = `url(${url}) ${hsx} ${hsy}, default`;
		Object.assign(pointer.style, {
			backgroundImage: `url(${url})`,
			width: `${cw}px`,
			height: `${ch}px`,
			margin: `${-hsy}px 0 0 ${-hsx}px`,
			transformOrigin: `${hsx}px ${hsy}px`,
		});
		pointer.classList.remove('none');
	}

	return {
		set(buf: ArrayBuffer) {
			if (buf.byteLength < 6) return;
			const v = new DataView(buf);
			const flags = v.getUint8(1);
			if (flags === 0x02) {
				const id = v.getUint32(2);
				activeId = id;
				const hit = cache.get(id);
				if (hit) {
					current = hit;
					apply();
				}
				return;
			}
			if (buf.byteLength < 14) return;
			const w = v.getUint16(2),
				h = v.getUint16(4),
				hx = v.getUint16(6),
				hy = v.getUint16(8),
				id = v.getUint32(10);
			activeId = id;
			if (!w || !h) {
				const empty: CachedCursor = {
					w: 0,
					h: 0,
					hx: 0,
					hy: 0,
					source: document.createElement('canvas'),
				};
				remember(id, empty);
				current = empty;
				apply();
				return;
			}
			if (flags === 0x01) {
				const png = new Blob([new Uint8Array(buf, 14)], { type: 'image/png' });
				createImageBitmap(png)
					.then((bitmap) => {
						const entry: CachedCursor = { w, h, hx, hy, source: bitmap };
						remember(id, entry);
						if (activeId === id) {
							current = entry;
							apply();
						}
					})
					.catch(() => {});
				return;
			}
			const src = document.createElement('canvas');
			src.width = w;
			src.height = h;
			src
				.getContext('2d')!
				.putImageData(
					new ImageData(new Uint8ClampedArray(buf, 14, w * h * 4), w, h),
					0,
					0
				);
			const entry: CachedCursor = { w, h, hx, hy, source: src };
			remember(id, entry);
			current = entry;
			apply();
		},
		apply,
	};
}
