/* planner.js - pure 3D path planning, no map dependency (testable in Node).
 * Local frame: x = east (m), y = north (m), z = altitude above ground (m). */
const Planner = (() => {
  const R = 6378137, D2R = Math.PI / 180;
  const toLocal = (o, [lng, lat]) => [(lng - o[0]) * D2R * R * Math.cos(o[1] * D2R), (lat - o[1]) * D2R * R];
  const toLngLat = (o, [x, y]) => [o[0] + x / (R * Math.cos(o[1] * D2R)) / D2R, o[1] + y / R / D2R];
  const bearing = (a, b) => (Math.atan2(b.x - a.x, b.y - a.y) / D2R + 360) % 360;
  const dist3 = (a, b) => Math.hypot(b.x - a.x, b.y - a.y, b.z - a.z);

  /* ---- Milestone 3a: binary min-heap, the priority queue behind A* ---- */
  class Heap {
    constructor() { this.a = []; }
    get size() { return this.a.length; }
    push(item, pri) {
      const a = this.a; a.push([pri, item]); let i = a.length - 1;
      while (i > 0) { const p = (i - 1) >> 1; if (a[p][0] <= a[i][0]) break; [a[p], a[i]] = [a[i], a[p]]; i = p; }
    }
    pop() {
      const a = this.a, top = a[0], last = a.pop();
      if (a.length) { a[0] = last; let i = 0;
        for (;;) { const l = 2 * i + 1, r = l + 1; let m = i;
          if (l < a.length && a[l][0] < a[m][0]) m = l;
          if (r < a.length && a[r][0] < a[m][0]) m = r;
          if (m === i) break; [a[m], a[i]] = [a[i], a[m]]; i = m; } }
      return top[1];
    }
  }

  const inPoly = (x, y, ring) => {
    let c = false;
    for (let i = 0, j = ring.length - 1; i < ring.length; j = i++) {
      const [xi, yi] = ring[i], [xj, yj] = ring[j];
      if ((yi > y) !== (yj > y) && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi) c = !c;
    }
    return c;
  };

  /* ---- Milestone 2: collision world. Buildings are rasterised to a height grid;
   * no-fly zones are infinite-height cylinders; start/goal pad cells are exempt. ---- */
  function buildWorld({ bounds, cell, clearance, buildings, zones, pads }) {
    const nx = Math.ceil((bounds.maxx - bounds.minx) / cell), ny = Math.ceil((bounds.maxy - bounds.miny) / cell);
    const H = new Float32Array(nx * ny);
    const ci = x => Math.min(nx - 1, Math.max(0, Math.floor((x - bounds.minx) / cell)));
    const cj = y => Math.min(ny - 1, Math.max(0, Math.floor((y - bounds.miny) / cell)));
    for (const b of buildings) {
      let x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
      for (const [x, y] of b.ring) { x0 = Math.min(x0, x); x1 = Math.max(x1, x); y0 = Math.min(y0, y); y1 = Math.max(y1, y); }
      if (x1 < bounds.minx || x0 > bounds.maxx || y1 < bounds.miny || y0 > bounds.maxy) continue;
      for (const [x, y] of b.ring) { const k = cj(y) * nx + ci(x); H[k] = Math.max(H[k], b.h); } // small buildings
      for (let j = cj(y0); j <= cj(y1); j++) for (let i = ci(x0); i <= ci(x1); i++)
        if (inPoly(bounds.minx + (i + .5) * cell, bounds.miny + (j + .5) * cell, b.ring)) H[j * nx + i] = Math.max(H[j * nx + i], b.h);
    }
    const inside = (x, y) => x >= bounds.minx && x <= bounds.maxx && y >= bounds.miny && y <= bounds.maxy;
    const pad = (x, y) => pads.some(p => ci(x) === ci(p.x) && cj(y) === cj(p.y));
    const hAt = (x, y) => H[cj(y) * nx + ci(x)];
    const hNear = (x, y) => { let m = 0; for (let dj = -1; dj <= 1; dj++) for (let di = -1; di <= 1; di++)
      m = Math.max(m, H[Math.min(ny - 1, Math.max(0, cj(y) + dj)) * nx + Math.min(nx - 1, Math.max(0, ci(x) + di))]); return m; };
    const zoneAt = (x, y, m = 0) => zones.find(z => Math.hypot(x - z.x, y - z.y) < z.r + m);
    // Rule: outside pads a point is flyable only if above roof + clearance buffer and outside every zone.
    const free = (x, y, z) => inside(x, y) && (pad(x, y) || (!zoneAt(x, y, 12) && z >= hAt(x, y) + clearance));
    const segFree = (a, b) => {
      const n = Math.max(1, Math.ceil(dist3(a, b) / (cell / 2)));
      for (let t = 0; t <= n; t++) { const f = t / n; if (!free(a.x + (b.x - a.x) * f, a.y + (b.y - a.y) * f, a.z + (b.z - a.z) * f)) return false; }
      return true;
    };
    return { bounds, cell, nx, ny, clearance, ci, cj, hAt, hNear, zoneAt, free, segFree, pad, zones,
             buildingCount: buildings.length };
  }

  /* ---- Milestone 3b: 3D A* on (x, y, altitude-level) lattice, 26-neighbour moves ---- */
  function plan(w, start, goal, { alts = [0, 30, 50, 70, 90, 110, 130, 150], maxPops = 600000 } = {}) {
    const t0 = Date.now(), { nx, ny, cell } = w, K = alts.length, N = nx * ny * K;
    const id = (i, j, k) => (k * ny + j) * nx + i;
    const pos = (i, j, k) => ({ x: w.bounds.minx + (i + .5) * cell, y: w.bounds.miny + (j + .5) * cell, z: alts[k] });
    const g = new Float32Array(N).fill(Infinity), came = new Int32Array(N).fill(-1), closed = new Uint8Array(N);
    const s = id(w.ci(start.x), w.cj(start.y), 0), e = id(w.ci(goal.x), w.cj(goal.y), 0);
    const gp = pos(w.ci(goal.x), w.cj(goal.y), 0), open = new Heap();
    g[s] = 0; open.push(s, 0); let pops = 0;
    while (open.size && pops++ < maxPops) {
      const c = open.pop(); if (closed[c]) continue; closed[c] = 1; if (c === e) break;
      const i = c % nx, j = ((c / nx) | 0) % ny, k = (c / (nx * ny)) | 0, a = pos(i, j, k);
      for (let dk = -1; dk <= 1; dk++) for (let dj = -1; dj <= 1; dj++) for (let di = -1; di <= 1; di++) {
        if (!di && !dj && !dk) continue;
        const ni = i + di, nj = j + dj, nk = k + dk;
        if (ni < 0 || nj < 0 || nk < 0 || ni >= nx || nj >= ny || nk >= K) continue;
        const n = id(ni, nj, nk); if (closed[n]) continue;
        const b = pos(ni, nj, nk);
        if (!w.free(b.x, b.y, b.z) || !w.free((a.x + b.x) / 2, (a.y + b.y) / 2, (a.z + b.z) / 2)) continue;
        let cost = dist3(a, b) * (dk ? 1.15 : 1);                       // climbing costs more energy
        if (!w.pad(b.x, b.y) && b.z - w.hAt(b.x, b.y) < w.clearance + 15) cost *= 1.3; // prefer wider margin over roofs
        const ng = g[c] + cost;
        if (ng < g[n]) { g[n] = ng; came[n] = c; open.push(n, ng + dist3(b, gp)); }
      }
    }
    if (came[e] < 0 && s !== e) return null;
    let raw = []; for (let c = e; c >= 0; c = came[c]) raw.push(pos(c % nx, ((c / nx) | 0) % ny, (c / (nx * ny)) | 0));
    raw.reverse(); raw[0] = { ...start, z: 0 }; raw[raw.length - 1] = { ...goal, z: 0 };
    // String-pulling: skip waypoints whenever a straight 3D segment is collision-free.
    const path = [raw[0]]; let i = 0;
    while (i < raw.length - 1) { let j = raw.length - 1; while (j > i + 1 && !w.segFree(raw[i], raw[j])) j--; path.push(raw[j]); i = j; }
    return { path, rawCount: raw.length, expanded: pops, ms: Date.now() - t0 };
  }

  /* Resample a path every `step` metres (with heading) for ribbons, metrics and animation. */
  function sample(path, step) {
    const out = []; let hx = path[path.length - 1].x - path[0].x, hy = path[path.length - 1].y - path[0].y;
    for (let i = 0; i < path.length - 1; i++) {
      const a = path[i], b = path[i + 1], L = dist3(a, b), n = Math.max(1, Math.ceil(L / step));
      if (Math.hypot(b.x - a.x, b.y - a.y) > 1) { hx = b.x - a.x; hy = b.y - a.y; }
      const d = Math.hypot(hx, hy) || 1;
      for (let t = 0; t < n; t++) { const f = t / n; out.push({ x: a.x + (b.x - a.x) * f, y: a.y + (b.y - a.y) * f, z: a.z + (b.z - a.z) * f, hx: hx / d, hy: hy / d }); }
    }
    const l = path[path.length - 1], d = Math.hypot(hx, hy) || 1; out.push({ ...l, hx: hx / d, hy: hy / d });
    return out;
  }

  /* ---- Dashboard numbers: distance, time, simulated battery, safety score ---- */
  function metrics(path, w, speed) {
    let dist = 0, climb = 0, minVert = Infinity, minNfz = Infinity;
    for (let i = 0; i < path.length - 1; i++) { dist += dist3(path[i], path[i + 1]); climb += Math.max(0, path[i + 1].z - path[i].z); }
    for (const p of sample(path, 10)) {
      if (p.z >= 25) minVert = Math.min(minVert, p.z - w.hAt(p.x, p.y));   // ignore take-off / landing
      for (const z of w.zones) minNfz = Math.min(minNfz, Math.hypot(p.x - z.x, p.y - z.y) - z.r);
    }
    const score = Math.round(Math.max(0, Math.min(100, 100 * Math.min(minVert / 40, minNfz / 40))));
    return { dist, seconds: dist / speed, battery: dist * 0.012 + climb * 0.06, minVert, minNfz, score };
  }

  /* ---- Milestone 6: turn the path into human-readable "why" statements ---- */
  function explain(path, w, res) {
    const log = [], direct = bearing(path[0], path[path.length - 1]), dir = (d) => d > 0 ? 'right' : 'left';
    const D = path[path.length - 1], crossed = w.zones.filter(z => {
      const dx = D.x, dy = D.y, t = Math.max(0, Math.min(1, (z.x * dx + z.y * dy) / (dx * dx + dy * dy)));
      return Math.hypot(z.x - dx * t, z.y - dy * t) < z.r; });
    let tall = 0; for (const p of sample([path[0], D].map(q => ({ ...q, z: 0 })), 15)) tall = Math.max(tall, w.hNear(p.x, p.y));
    log.push(`Scanned ${w.buildingCount} buildings. The straight line crosses ${crossed.length ? crossed.map(z => 'Zone ' + z.name).join(', ') : 'no restricted zones'} and buildings up to ${Math.round(tall)} m, so it was rejected.`);
    for (let i = 0; i < path.length - 1; i++) {
      const a = path[i], b = path[i + 1], dz = b.z - a.z, dh = Math.hypot(b.x - a.x, b.y - a.y), n = `Leg ${i + 1}: `;
      let obs = 0; for (const p of sample(path.slice(i, i + 3), 15)) obs = Math.max(obs, w.hNear(p.x, p.y)); // look 2 legs ahead
      if (dz >= 8) log.push(`${n}Ascended to ${Math.round(b.z)} m to clear local obstructions (tallest nearby roof ${Math.round(obs)} m + ${w.clearance} m safety buffer).`);
      else if (dz <= -8) log.push(b.z < 1 ? `${n}Descended safely onto the destination pad.` : `${n}Descended to ${Math.round(b.z)} m as roofs ahead got lower.`);
      else {
        const dev = ((bearing(a, b) - direct + 540) % 360) - 180;
        let near = null, nd = 1e9;
        for (const z of w.zones) for (const [p, q] of [[a, b], [b, path[Math.min(i + 2, path.length - 1)]]]) { // this leg and the next
          const vx = q.x - p.x, vy = q.y - p.y, t = Math.max(0, Math.min(1, ((z.x - p.x) * vx + (z.y - p.y) * vy) / (vx * vx + vy * vy || 1)));
          const d = Math.hypot(z.x - p.x - vx * t, z.y - p.y - vy * t) - z.r; if (d < nd) { nd = d; near = z; } }
        if (Math.abs(dev) > 8 && nd < 150) log.push(`${n}Diverted ${Math.round(Math.abs(dev))}° ${dir(dev)} of the direct course (heading ${Math.round(bearing(a, b))}°) to bypass Restricted Airspace Zone ${near.name}.`);
        else if (Math.abs(dev) > 8) log.push(`${n}Steered ${Math.round(Math.abs(dev))}° ${dir(dev)} (heading ${Math.round(bearing(a, b))}°) to stay clear of building masses.`);
        else if (nd < 40) log.push(`${n}Cruised ${Math.round(dh)} m at ${Math.round(a.z)} m while skirting the edge of Restricted Airspace Zone ${near.name}.`);
        else log.push(`${n}Cruised ${Math.round(dh)} m at ${Math.round(a.z)} m, close to the direct course.`);
      }
    }
    log.push(`Search: 3D A* expanded ${res.expanded.toLocaleString()} nodes in ${res.ms} ms; ${res.rawCount} grid steps were smoothed into ${path.length} waypoints.`);
    return log;
  }
  return { toLocal, toLngLat, bearing, buildWorld, plan, sample, metrics, explain };
})();
if (typeof module !== 'undefined') module.exports = Planner;
