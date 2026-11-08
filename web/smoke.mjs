// Headless check of web/aimtrk.wasm with the demo's renderer before it is
// published (pages workflow): no imports; the aim spot is found to a small
// fraction of a pixel; hot pixels, a sunlit window and glints are rejected;
// a person walking past makes no false spots.
//   node web/smoke.mjs
import { readFileSync } from 'node:fs';
import { createScene, renderFrame, setOption, W, H, NOMINAL_TOTAL_DN } from './render.js';

const mod = new WebAssembly.Module(readFileSync(new URL('./aimtrk.wasm', import.meta.url)));
const imports = WebAssembly.Module.imports(mod);
if (imports.length) throw new Error(`unexpected imports: ${JSON.stringify(imports)}`);
const A = new WebAssembly.Instance(mod, {}).exports;

let failures = 0;
const check = (ok, what) => {
	console.log(`${ok ? 'ok  ' : 'FAIL'} ${what}`);
	if (!ok) failures++;
};

check(A.web_init(W, H) > 0, 'detector init 1280 x 800');
const img = new Uint8Array(A.memory.buffer, A.web_frame(), W * H);
const sc = createScene(7);
const rnd = sc.rnd;

// Strobed like a weapon in slot 0 of 2: a spot in every other frame.
function run(frames, opts = {}) {
	const res = { found: 0, lit: 0, err: [], strays: 0, rejected: [0, 0, 0, 0] };
	for (let k = 0; k < frames; k++) {
		const lit = k % 2 === 0;
		const truth = { x: 100 + rnd() * (W - 200), y: 100 + rnd() * (H - 200), total: NOMINAL_TOTAL_DN };
		if (opts.avoidWindow && truth.x > 1000 && truth.y < 220) truth.x -= 400;
		renderFrame(sc, img, lit ? [truth] : []);
		const n = A.web_detect();
		const sp = new Float64Array(A.memory.buffer, A.web_spots(), 10 * Math.max(n, 1));
		const st = new Float64Array(A.memory.buffer, A.web_stats(), 14);
		if (st[13]) continue; // learning frame
		for (let r = 0; r < 4; r++) res.rejected[r] += st[5 + r];
		let matched = false;
		for (let i = 0; i < n; i++) {
			const d = Math.hypot(sp[10 * i] - truth.x, sp[10 * i + 1] - truth.y);
			if (lit && d < 3 && !matched) {
				matched = true;
				res.err.push(d);
			} else {
				res.strays++;
			}
		}
		if (lit) {
			res.lit++;
			if (matched) res.found++;
		}
	}
	return res;
}

const rms = (a) => Math.sqrt(a.reduce((s, x) => s + x * x, 0) / Math.max(a.length, 1));

let r = run(60);
check(r.found === r.lit, `quiet lane: ${r.found} of ${r.lit} spots found`);
check(rms(r.err) < 0.1, `quiet lane: position error ${rms(r.err).toFixed(3)} px RMS`);
check(r.strays === 0, `quiet lane: ${r.strays} false spots (hot pixels rejected as small: ${r.rejected[0]})`);

setOption(sc, 'window', true);
setOption(sc, 'glints', true);
setOption(sc, 'leak', 12);
r = run(60, { avoidWindow: true });
check(r.found === r.lit, `window, glints, 12 DN leak: ${r.found} of ${r.lit} spots found`);
check(r.strays === 0, `window, glints: ${r.strays} false spots (large ${r.rejected[1]}, elongated ${r.rejected[3]})`);
check(r.rejected[1] > 0, 'the sunlit window is rejected as too large');

setOption(sc, 'person', true);
r = run(120, { avoidWindow: true });
check(r.found >= r.lit - 1, `person walking past: ${r.found} of ${r.lit} spots found`);
check(r.strays === 0, `person walking past: ${r.strays} false spots`);

if (failures) {
	console.error(`${failures} check(s) failed`);
	process.exit(1);
}
console.log('web smoke test passed');
