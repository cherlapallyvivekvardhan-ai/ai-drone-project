/* app.js - Mapbox scene, no-fly zones, neon ribbon, drone animation, UI wiring. */
const CFG = __CONFIG__;
mapboxgl.accessToken = CFG.token;
const $ = id => document.getElementById(id), fc = f => ({ type: 'FeatureCollection', features: f || [] });
$('a').value = CFG.startAddress; $('b').value = CFG.destAddress;

/* ---------- Milestone 1: dual viewports ---------- */
const map = new mapboxgl.Map({ container: 'map', style: 'mapbox://styles/mapbox/dark-v11', center: CFG.center, zoom: 15.5, pitch: 60, bearing: -20, antialias: true });
const mini = new mapboxgl.Map({ container: 'mini', style: 'mapbox://styles/mapbox/streets-v12', center: CFG.center, zoom: 12, attributionControl: false });
const ready = m => new Promise(r => m.loaded() ? r() : m.once('load', r));

map.on('load', () => {
  map.addSource('dem', { type: 'raster-dem', url: 'mapbox://mapbox.mapbox-terrain-dem-v1', tileSize: 512, maxzoom: 14 });
  map.setTerrain({ source: 'dem', exaggeration: 1 });                       // real elevation
  map.addLayer({ id: 'bldg', source: 'composite', 'source-layer': 'building', type: 'fill-extrusion', minzoom: 14,
    filter: ['==', 'extrude', 'true'], paint: { 'fill-extrusion-color': '#3a4560', 'fill-extrusion-height': ['get', 'height'],
    'fill-extrusion-base': ['get', 'min_height'], 'fill-extrusion-opacity': 0.9 } }); // 3D buildings
  const ext = (id, color, op, emissive) => { map.addSource(id, { type: 'geojson', data: fc() });
    map.addLayer({ id, source: id, type: 'fill-extrusion', paint: { 'fill-extrusion-color': color, 'fill-extrusion-opacity': op,
      'fill-extrusion-height': ['get', 'top'], 'fill-extrusion-base': ['get', 'bot'], 'fill-extrusion-emissive-strength': emissive } }); };
  ext('zones', '#ff1f1f', 0.35, 0.8);   // Milestone 4: red no-fly cylinders
  ext('glow', '#00ffe1', 0.22, 1);      //              neon ribbon: soft halo ...
  ext('core', '#7dfff0', 1, 1);         //              ... and bright emissive core
  ext('drone', '#ff9f1c', 1, 1);
});

let O, S, G, zones = [], route = null, world = null, miniMarker, raf = 0, fpvMode = false;
const miniReady = ready(mini);
miniMarker = new mapboxgl.Marker({ color: '#ff9f1c', scale: 0.7 });

/* ---------- helpers ---------- */
const say = t => $('status').textContent = t;
async function geocode(q) {
  const r = await fetch(`https://api.mapbox.com/search/geocode/v6/forward?q=${encodeURIComponent(q)}&limit=1&access_token=${CFG.token}`);
  const j = await r.json(); if (!j.features?.length) throw new Error('Address not found: ' + q);
  return j.features[0].geometry.coordinates;
}
const waitIdle = () => Promise.race([new Promise(r => map.once('idle', r)), new Promise(r => setTimeout(r, 5000))]);
const poly = (pts, props) => ({ type: 'Feature', properties: props, geometry: { type: 'Polygon', coordinates: [[...pts, pts[0]].map(c => Planner.toLngLat(O, c))] } });
const circle = (z, n = 48) => Array.from({ length: n }, (_, i) => [z.x + z.r * Math.cos(2 * Math.PI * i / n), z.y + z.r * Math.sin(2 * Math.PI * i / n)]);

/* Milestone 4: 3 restricted cylinders placed on/near the straight A-B line so they must be bypassed */
function makeZones(L, ux, uy) {
  const r = Math.min(Math.max(60, L * 0.08), L * 0.14), nx = -uy, ny = ux;
  return [['Alpha', .30, 0], ['Bravo', .58, .7], ['Charlie', .80, -.6]].map(([name, t, off]) =>
    ({ name, r, x: ux * L * t + nx * r * off, y: uy * L * t + ny * r * off }));
}

/* Ribbon = chain of small extruded quads along the path (visible from any tilt, unlike a flat line) */
function ribbon(path, w, t) {
  const pts = Planner.sample(path, 12), f = [];
  for (let i = 0; i < pts.length - 1; i++) { const p = pts[i], q = pts[i + 1], d = Math.hypot(q.x - p.x, q.y - p.y);
    const ring = d < .5 ? [[p.x - w, p.y - w], [p.x + w, p.y - w], [p.x + w, p.y + w], [p.x - w, p.y + w]]
      : [[p.x - p.hy * w, p.y + p.hx * w], [q.x - p.hy * w, q.y + p.hx * w], [q.x + p.hy * w, q.y - p.hx * w], [p.x + p.hy * w, p.y - p.hx * w]];
    f.push(poly(ring, { bot: Math.max(0, Math.min(p.z, q.z) - t), top: Math.max(p.z, q.z) + t })); }
  return fc(f);
}

/* ---------- Plan Route ---------- */
$('plan').onclick = async () => {
  try {
    cancelAnimationFrame(raf); $('fly').disabled = $('cam').disabled = true; $('log').innerHTML = '';
    say('Geocoding…'); const [A, B] = await Promise.all([geocode($('a').value), geocode($('b').value)]);
    O = A; S = { x: 0, y: 0, z: 0 }; const [gx, gy] = Planner.toLocal(O, B); G = { x: gx, y: gy, z: 0 };
    const L = Math.hypot(gx, gy); if (L < 200 || L > 4000) throw new Error(`Distance ${Math.round(L)} m - choose points 200 m to 4 km apart.`);
    zones = makeZones(L, gx / L, gy / L);
    const pad = 250, xs = [0, gx, ...zones.flatMap(z => [z.x - z.r, z.x + z.r])], ys = [0, gy, ...zones.flatMap(z => [z.y - z.r, z.y + z.r])];
    const bounds = { minx: Math.min(...xs) - pad, maxx: Math.max(...xs) + pad, miny: Math.min(...ys) - pad, maxy: Math.max(...ys) + pad };

    say('Loading 3D buildings…');
    map.fitBounds([Planner.toLngLat(O, [bounds.minx, bounds.miny]), Planner.toLngLat(O, [bounds.maxx, bounds.maxy])],
      { padding: { top: 60, bottom: 60, left: 380, right: 60 }, pitch: 50, bearing: 0, duration: 0, maxZoom: 16 });
    await waitIdle();
    const buildings = [];
    for (const f of map.querySourceFeatures('composite', { sourceLayer: 'building' })) {
      if (f.properties.extrude === 'false') continue;
      const h = +f.properties.height || 8, g = f.geometry;
      const rings = g.type === 'Polygon' ? [g.coordinates[0]] : g.type === 'MultiPolygon' ? g.coordinates.map(p => p[0]) : [];
      for (const r of rings) buildings.push({ h, ring: r.map(c => Planner.toLocal(O, c)) });
    }
    if (!buildings.length) say('Warning: no building data loaded - zoom the map in and retry.');

    const cell = Math.max(30, Math.max(bounds.maxx - bounds.minx, bounds.maxy - bounds.miny) / 110);
    world = Planner.buildWorld({ bounds, cell, clearance: 10, buildings, zones, pads: [S, G] });
    say('Running 3D A*…'); await new Promise(r => setTimeout(r, 30));      // let the UI repaint
    const res = Planner.plan(world, S, G);
    if (!res) return say('No safe path found (buildings too tall or points enclosed by zones).');
    route = res.path;

    map.getSource('zones').setData(fc(zones.map(z => poly(circle(z), { bot: 0, top: 300 }))));
    map.getSource('glow').setData(ribbon(route, 7, 5)); map.getSource('core').setData(ribbon(route, 2.5, 2));
    map.getSource('drone').setData(fc());
    const m = Planner.metrics(route, world, CFG.speed);
    $('m-dist').textContent = (m.dist / 1000).toFixed(2) + ' km'; $('m-time').textContent = Math.floor(m.seconds / 60) + 'm ' + Math.round(m.seconds % 60) + 's';
    $('m-batt').textContent = m.battery.toFixed(1) + ' %'; $('m-clear').textContent = m.score + ' / 100';
    $('log').innerHTML = Planner.explain(route, world, res).map(t => `<li>${t}</li>`).join('');
    say(`Route ready - ${buildings.length} buildings, ${zones.length} no-fly zones.`);
    $('fly').disabled = $('cam').disabled = false; drawMini();
  } catch (e) { say('⚠ ' + e.message); }
};

/* 2D overview: bbox, zones, path, A/B and live drone marker */
async function drawMini() {
  await miniReady; const ll = c => Planner.toLngLat(O, c);
  ['route', 'zones2', 'ab'].forEach(id => { if (mini.getLayer(id)) mini.removeLayer(id); if (mini.getSource(id)) mini.removeSource(id); });
  mini.addSource('route', { type: 'geojson', data: { type: 'Feature', geometry: { type: 'LineString', coordinates: route.map(p => ll([p.x, p.y])) } } });
  mini.addLayer({ id: 'route', source: 'route', type: 'line', paint: { 'line-color': '#00b8a3', 'line-width': 4 } });
  mini.addSource('zones2', { type: 'geojson', data: fc(zones.map(z => poly(circle(z), {}))) });
  mini.addLayer({ id: 'zones2', source: 'zones2', type: 'fill', paint: { 'fill-color': '#ff1f1f', 'fill-opacity': .4 } });
  const b = world.bounds, a = ll([b.minx, b.miny]), c = ll([b.maxx, b.maxy]);
  mini.addSource('ab', { type: 'geojson', data: { type: 'Feature', geometry: { type: 'LineString', coordinates: [a, [c[0], a[1]], c, [a[0], c[1]], a] } } });
  mini.addLayer({ id: 'ab', source: 'ab', type: 'line', paint: { 'line-color': '#333', 'line-dasharray': [2, 2] } });
  mini._m = mini._m || []; mini._m.forEach(m => m.remove());
  mini._m = [new mapboxgl.Marker({ color: '#2ecc71', scale: .6 }).setLngLat(O).addTo(mini),
             new mapboxgl.Marker({ color: '#e74c3c', scale: .6 }).setLngLat(ll([G.x, G.y])).addTo(mini)];
  miniMarker.setLngLat(O).addTo(mini); mini.fitBounds([a, c], { padding: 12, duration: 0 });
}

/* ---------- Milestone 5: drone animation + camera modes ---------- */
const droneShape = (p) => { const f = [p.hx, p.hy], l = [-p.hy, p.hx];
  const pt = (fx, fy) => [p.x + f[0] * fx + l[0] * fy, p.y + f[1] * fx + l[1] * fy];
  return poly([pt(-7, -6), pt(11, 0), pt(-7, 6), pt(-3, 0)], { bot: p.z + 1, top: p.z + 6 }); };

function fpv(p) {   // camera 35 m behind the drone, looking 250 m ahead
  const back = Planner.toLngLat(O, [p.x - p.hx * 35, p.y - p.hy * 35]), ahead = Planner.toLngLat(O, [p.x + p.hx * 250, p.y + p.hy * 250]);
  const ground = map.queryTerrainElevation(back) || 0, cam = map.getFreeCameraOptions();
  cam.position = mapboxgl.MercatorCoordinate.fromLngLat(back, ground + p.z + 14);
  cam.lookAtPoint(ahead); map.setFreeCameraOptions(cam);
}
$('cam').onclick = () => {
  fpvMode = !fpvMode; $('cam').textContent = 'View: ' + (fpvMode ? 'FPV' : 'Orbit');
  if (!fpvMode) map.jumpTo({ pitch: 60, zoom: 16 });   // hand control back to free orbit
};
$('fly').onclick = () => {
  cancelAnimationFrame(raf); const pts = Planner.sample(route, 5), total = (pts.length - 1) * 5, dur = Math.max(10, total / 45) * 1000, t0 = performance.now();
  const tick = now => {
    const t = Math.min(1, (now - t0) / dur), p = pts[Math.round(t * (pts.length - 1))];
    map.getSource('drone').setData(fc([droneShape(p)])); miniMarker.setLngLat(Planner.toLngLat(O, [p.x, p.y]));
    if (fpvMode) fpv(p); if (t < 1) raf = requestAnimationFrame(tick);
  }; raf = requestAnimationFrame(tick);
};
