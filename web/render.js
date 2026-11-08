// IR camera frames for the browser demo: a JavaScript cut-down of sim/camsim.py
// (same sensor values; flat screen, no lens distortion). Used by app.js and
// by the headless smoke test (smoke.mjs).

export const W = 1280;
export const H = 800;

// sim/camsim.py Sensor (assumed values for a 1 MP global-shutter sensor)
const FULL_WELL_E = 10000;
const E_PER_DN = FULL_WELL_E / 255;
const READ_NOISE_E = 12;
const BLACK_DN = 4;
const DSNU_DN = 0.3;
const AMBIENT_DN = 12;
const AMBIENT_GRADIENT = 0.3;
const HOT_FRACTION = 2e-5;

export const SPOT_SIGMA_PX = 1.3; // 2.8 mm emitter spot at ~2.6 mm/px, plus the lens blur
export const NOMINAL_TOTAL_DN = 1500; // integrated pulse signal, DN * px (peak about 140 DN)

const NOISE_N = 1 << 21;

function mulberry32(seed) {
	let a = seed >>> 0;
	return () => {
		a = (a + 0x6d2b79f5) >>> 0;
		let t = a;
		t = Math.imul(t ^ (t >>> 15), t | 1);
		t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
		return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
	};
}

function gaussTable(n, rnd) {
	const g = new Float32Array(n);
	for (let i = 0; i < n; i += 2) {
		const u = Math.max(rnd(), 1e-12);
		const v = rnd();
		const r = Math.sqrt(-2 * Math.log(u));
		g[i] = r * Math.cos(2 * Math.PI * v);
		g[i + 1] = r * Math.sin(2 * Math.PI * v);
	}
	return g;
}

// erf (Abramowitz-Stegun 7.1.26, |error| < 1.5e-7)
function erf(x) {
	const s = Math.sign(x);
	const a = Math.abs(x);
	const t = 1 / (1 + 0.3275911 * a);
	const y = 1 - (((((1.061405429 * t - 1.453152027) * t) + 1.421413741) * t - 0.284496736) * t +
		0.254829592) * t * Math.exp(-a * a);
	return s * y;
}

// share of a 1-D Gaussian (centre c, sigma s) falling on pixel i (covers i-0.5..i+0.5)
function pixelShare(i, c, s) {
	const k = 1 / (Math.SQRT2 * s);
	return 0.5 * (erf((i + 0.5 - c) * k) - erf((i - 0.5 - c) * k));
}

export function createScene(seed = 1234) {
	const rnd = mulberry32(seed);
	const noise = gaussTable(NOISE_N, rnd);
	// fixed pattern: DSNU and hot pixels belong to the sensor
	const fixed = new Float32Array(W * H);
	for (let i = 0; i < fixed.length; i++) fixed[i] = DSNU_DN * noise[(i * 7) & (NOISE_N - 1)];
	const hot = [];
	const nHot = Math.round(W * H * HOT_FRACTION);
	for (let k = 0; k < nHot; k++) {
		const i = Math.floor(rnd() * W * H);
		fixed[i] += 40 + 160 * rnd();
		hot.push(i);
	}
	// what the projector shows (0..1): sky gradient, ground, two target silhouettes
	const scene = new Float32Array(W * H);
	for (let y = 0; y < H; y++) {
		for (let x = 0; x < W; x++) {
			let v = y < 470 ? 0.55 + 0.35 * (y / 470) : 0.35;
			for (const [tx, ty, tw, th] of [[300, 330, 70, 180], [860, 380, 55, 140]]) {
				const dx = (x - tx) / tw;
				const dy = (y - ty) / th;
				if (dx * dx + (dy < 0 ? dy * dy * 4 : 0) < 0.25 && dy > -0.5 && dy < 0.5) v = 0.12;
			}
			scene[y * W + x] = v;
		}
	}
	return {
		noise, fixed, scene, hot, rnd, frame: 0,
		base: new Float32Array(W * H), sig: new Float32Array(W * H),
		opts: { leak: 8, window: false, glints: false, person: false },
		personX: -200,
	};
}

// background level of every pixel (DN, before noise) for the current options
function updateBase(sc) {
	const { base, sig, fixed, scene, opts } = sc;
	const px = sc.personX;
	for (let y = 0; y < H; y++) {
		for (let x = 0; x < W; x++) {
			const i = y * W + x;
			let light = AMBIENT_DN * (1 + AMBIENT_GRADIENT * (x / W - 0.5)) + opts.leak * scene[i];
			if (opts.person) {
				// a person between the projector and the screen: an elliptic head and a body
				const hx = (x - px) / 38;
				const hy = (y - 250) / 48;
				const bx = (x - px) / 95;
				const by = (y - 520) / 260;
				if (hx * hx + hy * hy < 1 || (bx * bx < 1 && by * by < 1)) light *= 0.3;
			}
			if (opts.window && x > 1060 && y < 150) light += 260; // sunlit window: saturates
			base[i] = light;
			const e = light * E_PER_DN;
			sig[i] = Math.sqrt(e + READ_NOISE_E * READ_NOISE_E) / E_PER_DN;
		}
	}
	if (opts.glints) {
		// reflections off a rail: short thin streaks
		for (const [gx, gy, len] of [[180, 690, 18], [640, 705, 14], [1010, 680, 20]]) {
			for (let k = 0; k < len; k++) base[gy * W + gx + k] += 70;
		}
	}
}

/*
 * Render one frame into img (Uint8Array W*H). spots: [{x, y, total}] in
 * pixels (image coordinates, pixel centres at integers).
 */
export function renderFrame(sc, img, spots) {
	if (sc.opts.person) {
		sc.personX += 6;
		if (sc.personX > W + 200) sc.personX = -200;
	}
	if (sc.dirty || sc.opts.person || sc.frame === 0) {
		updateBase(sc);
		sc.dirty = false;
	}
	sc.frame++;
	const { base, sig, fixed, noise } = sc;
	const off = Math.floor(sc.rnd() * NOISE_N);
	const mask = NOISE_N - 1;
	for (let i = 0; i < W * H; i++) {
		const v = base[i] + sig[i] * noise[(i + off) & mask] + BLACK_DN + fixed[i];
		img[i] = v <= 0 ? 0 : v >= 255 ? 255 : Math.round(v);
	}
	for (const s of spots) {
		const r = Math.ceil(4 * SPOT_SIGMA_PX) + 1;
		const x0 = Math.max(0, Math.floor(s.x) - r);
		const x1 = Math.min(W - 1, Math.floor(s.x) + r);
		const y0 = Math.max(0, Math.floor(s.y) - r);
		const y1 = Math.min(H - 1, Math.floor(s.y) + r);
		const sx = [];
		for (let x = x0; x <= x1; x++) sx.push(pixelShare(x, s.x, SPOT_SIGMA_PX));
		for (let y = y0; y <= y1; y++) {
			const sy = pixelShare(y, s.y, SPOT_SIGMA_PX);
			for (let x = x0; x <= x1; x++) {
				const i = y * W + x;
				const light = base[i] + s.total * sx[x - x0] * sy;
				const e = light * E_PER_DN;
				const n = Math.sqrt(e + READ_NOISE_E * READ_NOISE_E) / E_PER_DN;
				const v = light + n * noise[(i * 13 + off) & mask] + BLACK_DN + fixed[i];
				img[i] = v <= 0 ? 0 : v >= 255 ? 255 : Math.round(v);
			}
		}
	}
}

export function setOption(sc, key, value) {
	sc.opts[key] = value;
	sc.dirty = true;
}
