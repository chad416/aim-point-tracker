// Browser demo: renders IR camera frames (render.js) and runs the aimtrk spot
// detector (web/aimtrk.wasm, the C library) on each one.
import { createScene, renderFrame, setOption, W, H } from './render.js';

const FRAMES_PER_S = 30; // the real camera runs at 100 fps; the page renders fewer
const ui = {};
let A;   // wasm exports
let img; // the frame, inside wasm memory
const sc = createScene();

const state = {
	aim: { x: W / 2, y: H / 2 }, lastMouse: -1e9, shotPending: false,
	brightness: 1500, frame: 0, acc: 0, last: 0,
	lit: null,      // last lit frame: { pixels, truth, spot, shot }
	errors: [], times: [], strays: 0, shots: [],
};

async function load() {
	let inst;
	try {
		inst = await WebAssembly.instantiateStreaming(fetch('aimtrk.wasm'), {});
	} catch (e) {
		inst = await WebAssembly.instantiate(await (await fetch('aimtrk.wasm')).arrayBuffer(), {});
	}
	A = inst.instance.exports;
	if (!A.web_init(W, H)) throw new Error('detector init failed');
	img = new Uint8Array(A.memory.buffer, A.web_frame(), W * H);
}

function autoAim(t) {
	// when the mouse is away: a slow sweep over the screen, like a shooter tracking a target
	return { x: W / 2 + 0.36 * W * Math.sin(t * 0.31), y: H / 2 + 0.33 * H * Math.sin(t * 0.47 + 1) };
}

// one camera frame: render, detect, match against the truth
function step() {
	const t = state.frame / FRAMES_PER_S;
	const lit = state.frame % 2 === 0; // slot 0 of 2: the emitter flashes in every other frame
	state.frame++;
	const aim = performance.now() - state.lastMouse < 3000 ? state.aim : autoAim(t);
	const shot = lit && state.shotPending;
	if (shot) state.shotPending = false;
	const truth = { x: aim.x, y: aim.y, total: state.brightness * (shot ? 2 : 1) };
	renderFrame(sc, img, lit ? [truth] : []);

	const t0 = performance.now();
	const n = A.web_detect();
	const dt = performance.now() - t0;
	state.times.push(dt);
	if (state.times.length > 60) state.times.shift();

	const sp = new Float64Array(A.memory.buffer, A.web_spots(), 10 * Math.max(n, 1));
	const st = new Float64Array(A.memory.buffer, A.web_stats(), 14).slice();
	let spot = null;
	const others = [];
	for (let i = 0; i < n; i++) {
		const s = { x: sp[10 * i], y: sp[10 * i + 1], snr: sp[10 * i + 4], flags: sp[10 * i + 9] };
		if (lit && !spot && Math.hypot(s.x - truth.x, s.y - truth.y) < 3) spot = s;
		else others.push(s);
	}
	if (!st[13]) state.strays += others.length;
	if (lit) {
		if (spot) {
			const e = Math.hypot(spot.x - truth.x, spot.y - truth.y);
			state.errors.push(e);
			if (state.errors.length > 200) state.errors.shift();
			if (shot) state.shots.unshift(e);
		} else if (shot) {
			state.shots.unshift(null);
		}
		state.shots.length = Math.min(state.shots.length, 5);
		state.lit = { pixels: img.slice(), truth, spot, others, shot, stats: st };
	} else if (state.lit) {
		state.lit.others = state.lit.others.concat(others);
	}
	state.stats = st;
}

// ---- drawing --------------------------------------------------------------------

const css = (n) => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const LUT = new Uint8ClampedArray(256);
for (let v = 0; v < 256; v++) LUT[v] = 255 * Math.min(1, Math.pow(v / 160, 0.5));

let camImage;
function drawCamera() {
	const c = ui.cam;
	const ctx = c.getContext('2d');
	const L = state.lit;
	if (!L) return;
	if (!camImage) camImage = ctx.createImageData(W, H);
	const d = camImage.data;
	const p = L.pixels;
	for (let i = 0, j = 0; i < p.length; i++, j += 4) {
		const v = LUT[p[i]];
		d[j] = v;
		d[j + 1] = v;
		d[j + 2] = v;
		d[j + 3] = 255;
	}
	ctx.putImageData(camImage, 0, 0);

	// truth: a small cross; detection: a ring (image px; the canvas is shown at ~70 %)
	ctx.lineWidth = 2;
	ctx.strokeStyle = css('--muted');
	cross(ctx, L.truth.x, L.truth.y, 14);
	ctx.strokeStyle = css('--accent');
	ctx.lineWidth = 3;
	if (L.spot) ring(ctx, L.spot.x, L.spot.y, L.shot ? 22 : 14);
	ctx.strokeStyle = css('--crit');
	for (const s of L.others) ring(ctx, s.x, s.y, 12);
	if (L.shot) {
		ctx.fillStyle = css('--accent');
		ctx.font = '26px system-ui, sans-serif';
		ctx.fillText(L.spot ? 'shot' : 'shot: no spot', L.truth.x + 26, L.truth.y - 20);
	}
}

function cross(ctx, x, y, r) {
	ctx.beginPath();
	ctx.moveTo(x - r, y);
	ctx.lineTo(x + r, y);
	ctx.moveTo(x, y - r);
	ctx.lineTo(x, y + r);
	ctx.stroke();
}

function ring(ctx, x, y, r) {
	ctx.beginPath();
	ctx.arc(x, y, r, 0, 2 * Math.PI);
	ctx.stroke();
}

function drawMagnifier() {
	const c = ui.mag;
	const ctx = c.getContext('2d');
	const L = state.lit;
	if (!L) return;
	const N = 16;
	const k = c.width / N;
	const x0 = Math.round(L.truth.x) - N / 2;
	const y0 = Math.round(L.truth.y) - N / 2;
	for (let y = 0; y < N; y++) {
		for (let x = 0; x < N; x++) {
			const px = x0 + x;
			const py = y0 + y;
			const v = (px >= 0 && py >= 0 && px < W && py < H) ? LUT[L.pixels[py * W + px]] : 0;
			ctx.fillStyle = `rgb(${v},${v},${v})`;
			ctx.fillRect(x * k, y * k, k, k);
		}
	}
	// pixel (x0, y0) covers x0 - 0.5 .. x0 + 0.5
	const X = (x) => (x - x0 + 0.5) * k;
	const Y = (y) => (y - y0 + 0.5) * k;
	ctx.lineWidth = 2;
	ctx.strokeStyle = css('--muted');
	cross(ctx, X(L.truth.x), Y(L.truth.y), k * 1.5);
	if (L.spot) {
		ctx.strokeStyle = css('--accent');
		ring(ctx, X(L.spot.x), Y(L.spot.y), k * 0.6);
	}
}

function fmt(n, d = 0) {
	return Number.isFinite(n) ? n.toFixed(d) : '–';
}

function updatePanel() {
	const L = state.lit;
	const st = state.stats;
	if (!L || !st) return;
	const e = L.spot ? Math.hypot(L.spot.x - L.truth.x, L.spot.y - L.truth.y) : NaN;
	ui.found.textContent = L.spot ? 'yes' : 'no';
	ui.err.textContent = L.spot ? `${e.toFixed(3)} px (≈ ${(e * 2.6).toFixed(2)} mm)` : '–';
	const rms = Math.sqrt(state.errors.reduce((s, x) => s + x * x, 0) / Math.max(state.errors.length, 1));
	ui.rms.textContent = state.errors.length ? `${rms.toFixed(3)} px` : '–';
	ui.snr.textContent = L.spot ? fmt(L.spot.snr) : '–';
	const ts = state.times.slice().sort((a, b) => a - b);
	ui.ms.textContent = ts.length ? `${ts[Math.floor(ts.length / 2)].toFixed(2)} ms median` : '–';
	ui.sigma.textContent = `${st[0].toFixed(2)} DN / ${st[1].toFixed(1)} DN`;
	ui.blobs.textContent = `${st[2]} / ${st[3]} / ${st[4]}`;
	ui.rej_small.textContent = st[5];
	ui.rej_large.textContent = st[6];
	ui.rej_weak.textContent = st[7];
	ui.rej_elong.textContent = st[8];
	ui.snapped.textContent = st[10];
	ui.strays.textContent = state.strays;
	ui.shots.textContent = state.shots.length ?
		'Last shots: ' + state.shots.map((s) => (s === null ? 'missed' : `${s.toFixed(3)} px`)).join(' · ') :
		'Click the camera image to fire a shot.';
}

function frame(now) {
	const dt = state.last ? (now - state.last) / 1000 : 0;
	state.last = now;
	state.acc = Math.min(state.acc + dt, 0.2);
	while (state.acc >= 1 / FRAMES_PER_S) {
		state.acc -= 1 / FRAMES_PER_S;
		step();
	}
	drawCamera();
	drawMagnifier();
	updatePanel();
	requestAnimationFrame(frame);
}

function bind() {
	for (const id of ['cam', 'mag', 'found', 'err', 'rms', 'snr', 'ms', 'shots', 'leak', 'leak_v', 'bright',
		'bright_v', 'window', 'glints', 'person', 'sigma', 'blobs', 'rej_small', 'rej_large', 'rej_weak',
		'rej_elong', 'snapped', 'strays']) {
		ui[id] = document.getElementById(id);
	}
	const toImage = (e) => {
		const r = ui.cam.getBoundingClientRect();
		return { x: ((e.clientX - r.left) / r.width) * W - 0.5, y: ((e.clientY - r.top) / r.height) * H - 0.5 };
	};
	ui.cam.addEventListener('pointermove', (e) => {
		state.aim = toImage(e);
		state.lastMouse = performance.now();
	});
	ui.cam.addEventListener('pointerdown', (e) => {
		state.aim = toImage(e);
		state.lastMouse = performance.now();
		state.shotPending = true;
	});
	const slider = (el, out, f, apply) => {
		const upd = () => {
			out.textContent = f(Number(el.value));
			apply(Number(el.value));
		};
		el.addEventListener('input', upd);
		upd();
	};
	slider(ui.leak, ui.leak_v, (v) => `${v} DN`, (v) => setOption(sc, 'leak', v));
	slider(ui.bright, ui.bright_v, (v) => `${v} DN·px (peak ≈ ${Math.round(v / (2 * Math.PI * 1.69))} DN)`,
		(v) => { state.brightness = v; });
	for (const k of ['window', 'glints', 'person']) {
		ui[k].addEventListener('change', () => setOption(sc, k, ui[k].checked));
	}
}

(async () => {
	try {
		await load();
	} catch (e) {
		document.querySelector('header p').textContent = `Could not load the WebAssembly module: ${e.message}`;
		return;
	}
	bind();
	window.demo = { step, drawCamera, drawMagnifier, updatePanel, state }; // for automated checks
	requestAnimationFrame(frame);
})();
