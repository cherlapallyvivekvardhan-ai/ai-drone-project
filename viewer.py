"""Builds the animated 3D flight viewer (deck.gl + MapLibre) as one HTML string.

The animation runs in the browser, so playing, pausing and scrubbing are smooth
and never re-run the Streamlit script.
"""
import json

TEMPLATE = r"""<!DOCTYPE html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<link href="https://unpkg.com/maplibre-gl@3.6.2/dist/maplibre-gl.css" rel="stylesheet">
<script src="https://unpkg.com/maplibre-gl@3.6.2/dist/maplibre-gl.js"></script>
<script src="https://unpkg.com/deck.gl@8.9.35/dist.min.js"></script>
<style>
html,body{margin:0;height:100%;font:13px system-ui,-apple-system,"Segoe UI",sans-serif;background:#0b1220;color:#e8eef5}
#map{position:absolute;inset:0}
.panel{position:absolute;background:rgba(11,18,32,.82);border:1px solid rgba(255,255,255,.14);border-radius:10px;padding:10px 12px;backdrop-filter:blur(6px)}
#hud{top:12px;left:12px;min-width:190px}
#hud b{display:block;font-size:11px;font-weight:600;color:#93a4b8;margin-top:6px}
#hud span{font-size:17px;font-variant-numeric:tabular-nums}
#state{font-size:14px;font-weight:700;color:#5ce1e6}
#banner{top:12px;left:50%;transform:translateX(-50%);display:none;font-weight:700;color:#ffb347}
#bar{left:12px;right:12px;bottom:12px;display:flex;gap:10px;align-items:center;flex-wrap:wrap}
#bar button,#bar select{background:#1d2b44;color:#fff;border:1px solid #38507a;border-radius:6px;padding:6px 12px;font:inherit;cursor:pointer}
#bar button:hover{background:#27406a}
#bar button:focus-visible,#bar select:focus-visible,#seek:focus-visible{outline:2px solid #5ce1e6;outline-offset:2px}
#seek{flex:1;min-width:160px;accent-color:#5ce1e6}
label{display:flex;gap:6px;align-items:center;cursor:pointer}
#err{position:absolute;inset:0;display:none;align-items:center;justify-content:center;text-align:center;padding:30px;background:#0b1220;color:#ffb3b3}
.legend{top:12px;right:56px;font-size:12px;line-height:1.7}
.legend i{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:6px}
</style></head><body>
<div id="map"></div>
<div id="err"></div>
<div class="panel" id="hud">
  <div id="state">Ready</div>
  <b>Step</b><span id="h_step">0</span>
  <b>Altitude</b><span id="h_alt">0 m</span>
  <b>Distance flown</b><span id="h_dist">0 m</span>
  <b>Energy used</b><span id="h_en">0.00 Wh</span>
  <b>Obstacle risk</b><span id="h_risk">0%</span>
</div>
<div class="panel" id="banner"></div>
<div class="panel legend">
  <div><i style="background:#4682ff"></i>Planned route</div>
  <div><i style="background:#ff4646"></i>Flown path</div>
  <div><i style="background:#ffa500"></i>Re-route point</div>
  <div><i style="background:#aa00dc"></i>Obstacle</div>
  <div><i style="background:#ff8c00"></i>No-fly zone</div>
</div>
<div class="panel" id="bar">
  <button id="play">Play flight</button>
  <button id="restart">Restart</button>
  <input id="seek" type="range" min="0" max="1000" value="0" aria-label="Flight progress">
  <select id="speed" aria-label="Speed"><option value="0.5">0.5x</option><option value="1" selected>1x</option><option value="2">2x</option><option value="4">4x</option></select>
  <label><input type="checkbox" id="follow"> Chase camera</label>
  <select id="style" aria-label="Map style"><option value="dark">Dark map</option><option value="light">Light map</option><option value="satellite">Satellite</option></select>
</div>
<script>
const P = __PAYLOAD__;
const $ = id => document.getElementById(id);
function fail(msg){ const e=$('err'); e.style.display='flex'; e.textContent=msg; }
try {
  if (!window.maplibregl || !window.deck || !deck.MapboxOverlay)
    throw new Error('The 3D libraries could not load. Check your internet connection and reload.');

  const STYLES = {
    dark: 'https://basemaps.cartocdn.com/gl/dark-matter-gl-style/style.json',
    light: 'https://basemaps.cartocdn.com/gl/positron-gl-style/style.json',
    satellite: {version: 8,
      sources: {esri: {type: 'raster', tileSize: 256, maxzoom: 19,
        tiles: ['https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}'],
        attribution: 'Imagery: Esri, Maxar, Earthstar Geographics'}},
      layers: [{id: 'esri', type: 'raster', source: 'esri'}]}
  };
  $('style').value = P.style;

  const map = new maplibregl.Map({container: 'map', style: STYLES[P.style], center: P.view.center,
    zoom: P.view.zoom, pitch: P.view.pitch, bearing: P.view.bearing, maxPitch: 80, antialias: true});
  map.addControl(new maplibregl.NavigationControl({visualizePitch: true}), 'top-right');
  const overlay = new deck.MapboxOverlay({interleaved: false, layers: []});
  map.addControl(overlay);

  const N = P.frames.length, BASE_FPS = 8;
  let f = 0, playing = false, last = null, dirty = true;

  const lerp = (a, b, t) => [a[0] + (b[0]-a[0])*t, a[1] + (b[1]-a[1])*t, a[2] + (b[2]-a[2])*t];
  const at = (arr, x) => { const i = Math.min(Math.floor(x), arr.length-1), j = Math.min(i+1, arr.length-1);
                           return lerp(arr[i], arr[j], x - i); };
  const hmin = P.hmin, hmax = P.hmax;
  const heightColor = h => { const t = hmax > hmin ? (h-hmin)/(hmax-hmin) : 0, a = [70,165,190], b = [238,118,42];
    return [a[0]+(b[0]-a[0])*t, a[1]+(b[1]-a[1])*t, a[2]+(b[2]-a[2])*t, 235]; };

  const bLayer = new deck.PolygonLayer({id: 'buildings', data: P.buildings, extruded: true, pickable: true,
    getPolygon: d => d.p, getElevation: d => d.h, getFillColor: d => heightColor(d.h),
    getLineColor: [20,30,40,120], material: {ambient: .45, diffuse: .7, shininess: 24, specularColor: [60,64,70]}});
  const nLayer = P.nofly ? new deck.PolygonLayer({id: 'nofly', data: [P.nofly], extruded: true, wireframe: true,
    getPolygon: d => d.p, getElevation: d => d.h, getFillColor: [255,140,0,55], getLineColor: [255,140,0,220],
    lineWidthMinPixels: 2}) : null;
  const pathLayer = new deck.PathLayer({id: 'planned', data: [{p: P.planned}], getPath: d => d.p,
    getColor: [70,130,255,210], widthMinPixels: 3, capRounded: true, jointRounded: true});
  const ghostLayer = new deck.PathLayer({id: 'ghost', data: [{p: P.frames}], getPath: d => d.p,
    getColor: [255,70,70,55], widthMinPixels: 2});
  const pins = [{p: [P.start[1], P.start[0]], c: [0,190,0], h: P.startAlt + 70, t: 'START'},
                {p: [P.goal[1], P.goal[0]], c: [235,185,0], h: P.goalAlt + 70, t: 'GOAL'}];
  const pinLayer = new deck.ColumnLayer({id: 'pins', data: pins, diskResolution: 12, radius: 5, extruded: true,
    getPosition: d => d.p, getFillColor: d => d.c, getElevation: d => d.h});
  const txtLayer = new deck.TextLayer({id: 'pin-text', data: pins, getPosition: d => [d.p[0], d.p[1], d.h + 12],
    getText: d => d.t, getSize: 16, getColor: [255,255,255], background: true,
    getBackgroundColor: [0,0,0,160], backgroundPadding: [5,3], billboard: true});

  function render() {
    const i = Math.min(N-1, Math.floor(f)), pos = at(P.frames, f);
    const trail = P.frames.slice(0, i+1); trail.push(pos);
    const dyn = P.dyn.filter(o => f >= o.appear).map(o => ({p: at(o.pos, f), r: o.r}));
    const marks = P.marks.filter(m => f >= m.f).map(m => ({p: m.pos}));
    const layers = [bLayer]; if (nLayer) layers.push(nLayer);
    layers.push(pathLayer, ghostLayer,
      new deck.PathLayer({id: 'trail', data: [{p: trail}], getPath: d => d.p, getColor: [255,70,70,255],
        widthMinPixels: 5, capRounded: true, jointRounded: true}),
      new deck.LineLayer({id: 'drop', data: [{s: pos, t: [pos[0], pos[1], 0]}], getSourcePosition: d => d.s,
        getTargetPosition: d => d.t, getColor: [200,220,255,170], getWidth: 2}),
      new deck.ScatterplotLayer({id: 'shadow', data: [{p: [pos[0], pos[1], 0.5]}], getPosition: d => d.p,
        getRadius: 11, radiusUnits: 'meters', getFillColor: [0,0,0,150]}),
      new deck.ScatterplotLayer({id: 'marks', data: marks, getPosition: d => d.p, getRadius: 10,
        radiusUnits: 'meters', radiusMinPixels: 5, getFillColor: [255,165,0]}),
      new deck.ScatterplotLayer({id: 'dyn', data: dyn, getPosition: d => d.p, getRadius: d => d.r,
        radiusUnits: 'meters', radiusMinPixels: 6, getFillColor: [170,0,220,190]}),
      pinLayer, txtLayer,
      new deck.ScatterplotLayer({id: 'drone-glow', data: [{p: pos}], getPosition: d => d.p, getRadius: 22,
        radiusUnits: 'meters', radiusMinPixels: 12, getFillColor: [0,230,255,70]}),
      new deck.ScatterplotLayer({id: 'drone', data: [{p: pos}], getPosition: d => d.p, getRadius: 9,
        radiusUnits: 'meters', radiusMinPixels: 6, stroked: true, getFillColor: [0,230,255],
        getLineColor: [255,255,255], lineWidthMinPixels: 2}));
    overlay.setProps({layers});
    if ($('follow').checked) map.jumpTo({center: [pos[0], pos[1]]});

    const k = Math.min(N-1, Math.round(f)), ev = P.events[k];
    const dz = (P.frames[Math.min(N-1, k+1)][2] - P.frames[Math.max(0, k-1)][2]);
    $('h_step').textContent = k + ' / ' + (N-1);
    $('h_alt').textContent = P.hud.alt[k].toFixed(0) + ' m';
    $('h_dist').textContent = P.hud.dist[k].toFixed(0) + ' m';
    $('h_en').textContent = P.hud.energy[k].toFixed(2) + ' Wh';
    $('h_risk').textContent = Math.round(P.hud.risk[k] * 100) + '%';
    let s = k >= N-1 ? 'Delivered' : (dz > 0.5 ? 'Climbing' : (dz < -0.5 ? 'Descending' : 'Cruising'));
    if (k === 0 && !playing) s = 'Ready';
    const b = $('banner');
    if (ev === 1) { s = 'Re-routing'; b.textContent = 'New obstacle detected. Re-planning the route.'; b.style.display = 'block'; }
    else if (ev === 2) { s = 'Holding'; b.textContent = 'Path blocked. Holding position.'; b.style.display = 'block'; }
    else b.style.display = 'none';
    $('state').textContent = s;
    $('seek').value = N > 1 ? Math.round(f / (N-1) * 1000) : 0;
    $('play').textContent = playing ? 'Pause' : (f >= N-1 ? 'Replay' : 'Play flight');
    dirty = false;
  }

  function tick(ts) {
    if (playing) {
      if (last !== null) {
        f += (ts - last) / 1000 * BASE_FPS * Number($('speed').value);
        if (f >= N-1) { f = N-1; playing = false; }
        dirty = true;
      }
      last = ts;
    } else last = null;
    if (dirty) render();
    requestAnimationFrame(tick);
  }

  $('play').onclick = () => { if (f >= N-1) f = 0; playing = !playing; dirty = true; };
  $('restart').onclick = () => { f = 0; playing = false; dirty = true; };
  $('seek').oninput = e => { playing = false; f = Number(e.target.value) / 1000 * (N-1); dirty = true; };
  $('follow').onchange = () => { dirty = true; };
  $('style').onchange = e => map.setStyle(STYLES[e.target.value]);
  map.on('load', () => { dirty = true; });
  requestAnimationFrame(tick);
} catch (e) { fail(e.message || String(e)); }
</script></body></html>
"""


def build_viewer_html(res, style="dark"):
    """res is the dict returned by sim3d.run_simulation."""
    heights = [b["h"] for b in res["buildings"]] or [0, 1]
    trace = res["trace"]
    payload = {
        "view": res["view"],
        "style": style,
        "frames": res["frames"],
        "planned": res["planned"],
        "buildings": res["buildings"],
        "nofly": res["nofly"],
        "dyn": res["dyn"],
        "events": res["events"],
        "marks": res["reroute_marks"],
        "start": res["start"],
        "goal": res["goal"],
        "startAlt": res["start_alt"],
        "goalAlt": res["goal_alt"],
        "hmin": min(heights),
        "hmax": max(heights),
        "hud": {
            "alt": [round(t["alt_m"], 1) for t in trace],
            "energy": [round(t["cum_energy_wh"], 3) for t in trace],
            "dist": [round(t["cum_dist_m"], 1) for t in trace],
            "risk": [round(t["risk"], 3) for t in trace],
        },
    }
    text = json.dumps(payload, separators=(",", ":")).replace("</", "<\\/")
    return TEMPLATE.replace("__PAYLOAD__", text)
