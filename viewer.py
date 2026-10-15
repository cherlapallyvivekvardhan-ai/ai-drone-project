"""viewer.py - builds the animated 3D viewer as ONE self-contained HTML string.

Rendering: MapLibre GL JS 3.6.2 (base map) + deck.gl 8.9.35 (3D layers), both loaded from unpkg.
If either library fails to load, the page shows a friendly message and the whole information
panel (distances, route legs, events, step-by-step explanation, playback) keeps working.

Coordinates: every position in the payload is [lon, lat, z] where z is metres above ground level.
Frames carry lon/lat/z computed by sim3d from the same local frame as buildings and routes, and the
browser interpolates lon/lat/z linearly in time (exact, because lon/lat are linear in local x/y).

Run `python viewer.py --selftest` to check the HTML, JSON embedding and the JS interpolation core.
"""
from __future__ import annotations

import json
import re

MAPLIBRE = "https://unpkg.com/maplibre-gl@3.6.2/dist/maplibre-gl.js"
MAPLIBRE_CSS = "https://unpkg.com/maplibre-gl@3.6.2/dist/maplibre-gl.css"
DECKGL = "https://unpkg.com/deck.gl@8.9.35/dist.min.js"


def safe_json(obj) -> str:
    """JSON that is safe inside a <script> element (no </script>, no HTML comments, no U+2028/9)."""
    s = json.dumps(obj, separators=(",", ":"), ensure_ascii=True, allow_nan=False)
    return s.replace("<", "\\u003c").replace(">", "\\u003e").replace("&", "\\u0026")


def _round(o, nd=6):
    if isinstance(o, float):
        return round(o, nd)
    if isinstance(o, list):
        return [_round(v, nd) for v in o]
    if isinstance(o, tuple):
        return [_round(v, nd) for v in o]
    if isinstance(o, dict):
        return {k: _round(v, nd) for k, v in o.items()}
    return o


def payload(result: dict) -> dict:
    """The subset of the simulation result the browser needs (full precision for positions)."""
    keep = ("meta", "metrics", "frames", "routes", "paths", "buildings", "nofly", "dynamic", "events",
            "traces", "camera", "start", "dest", "buildings_source", "buildings_note", "failure", "warnings")
    p = {k: result[k] for k in keep if k in result}
    p["routes"] = [{k: v for k, v in r.items() if k != "waypoints"} | {"waypoints": r["waypoints"]}
                   for r in result["routes"]]
    p["units"] = {"distance": "m", "altitude": "m above ground level (AGL)", "energy": "Wh"}
    return _round(p, 9)


CORE_JS = r"""
// <CORE>
function lerp(a, b, u) { return a + (b - a) * u; }
function findIndex(frames, t) {            // largest i with frames[i].t <= t
  let lo = 0, hi = frames.length - 1;
  if (t <= frames[0].t) return 0;
  if (t >= frames[hi].t) return hi;
  while (hi - lo > 1) { const m = (lo + hi) >> 1; if (frames[m].t <= t) lo = m; else hi = m; }
  return lo;
}
const NUM_KEYS = ['lon', 'lat', 'z', 'x', 'y', 'dist', 'rem', 'e_used', 'e_left', 'batt', 'clr'];
function sampleAt(frames, t) {
  const i = findIndex(frames, t);
  const a = frames[i], b = frames[Math.min(i + 1, frames.length - 1)];
  const span = b.t - a.t;
  const u = span > 1e-9 ? Math.min(1, Math.max(0, (t - a.t) / span)) : 0;
  const o = { i: i, u: u, a: a, b: b, t: t };
  for (const k of NUM_KEYS) o[k] = lerp(a[k], b[k], u);
  return o;                                  // state/text/leg/ver describe the interval that STARTS at frame a
}
function localToGeo(meta, x, y) { return [meta.origin_lon + x / meta.m_per_deg_lon, meta.origin_lat + y / meta.m_per_deg_lat]; }
function geoToLocal(meta, lon, lat) { return [(lon - meta.origin_lon) * meta.m_per_deg_lon, (lat - meta.origin_lat) * meta.m_per_deg_lat]; }
function circlePolygon(meta, lon, lat, r, n) {
  const ring = [];
  for (let k = 0; k < n; k++) {
    const a = 2 * Math.PI * k / n;
    ring.push([lon + r * Math.cos(a) / meta.m_per_deg_lon, lat + r * Math.sin(a) / meta.m_per_deg_lat]);
  }
  ring.push(ring[0]);
  return ring;
}
function droneGeometry(s, meta, shadowR) {   // marker, vertical drop line and ground shadow share ONE lon/lat
  return { pos: [s.lon, s.lat, s.z], drop: [[s.lon, s.lat, 0], [s.lon, s.lat, s.z]],
           shadow: circlePolygon(meta, s.lon, s.lat, shadowR, 32) };
}
function obstaclePos(ob, t, meta) {          // moving obstacle: straight line in local metres
  const x = ob.p0[0] + ob.v[0] * t, y = ob.p0[1] + ob.v[1] * t;
  const g = localToGeo(meta, x, y);
  return [g[0], g[1], ob.p0[2] + ob.v[2] * t];
}
function fmtM(v) { return (Math.round(v * 10) / 10).toFixed(1) + ' m'; }
// </CORE>
"""

HTML = r"""<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<link rel="stylesheet" href="__MAPLIBRE_CSS__">
<style>
:root{--bg:#0f1420;--panel:#171d2c;--line:#2a3350;--txt:#e6eaf5;--mut:#9aa6c4;--cy:#22d3ee;--gr:#4ade80;--or:#fb923c;--pu:#c084fc;--rd:#f87171;--ye:#facc15}
*{box-sizing:border-box}html,body{margin:0;height:100%;background:var(--bg);color:var(--txt);font:13px/1.4 system-ui,Segoe UI,Roboto,sans-serif}
#app{display:flex;height:100%;min-height:600px}
#left{flex:1;display:flex;flex-direction:column;min-width:0}
#mapwrap{position:relative;flex:1;min-height:300px;background:#1b2338}
#map{position:absolute;inset:0}
#maperr{position:absolute;inset:0;display:none;align-items:center;justify-content:center;text-align:center;padding:24px;background:#1b2338;z-index:5}
#maperr>div{max-width:460px;border:1px solid var(--or);border-radius:10px;padding:16px;background:#241d1a}
#hud{position:absolute;left:10px;top:10px;z-index:4;max-width:360px;background:rgba(15,20,32,.88);border:1px solid var(--line);border-radius:10px;padding:10px 12px}
#badge{display:inline-block;padding:2px 9px;border-radius:99px;font-weight:700;font-size:12px;background:#334;color:#fff}
#htext{margin-top:6px;color:var(--txt)} .kv{display:grid;grid-template-columns:auto 1fr;gap:1px 10px;margin-top:6px;font-variant-numeric:tabular-nums}.kv span:nth-child(odd){color:var(--mut)}
#legend{position:absolute;left:10px;bottom:10px;z-index:4;background:rgba(15,20,32,.88);border:1px solid var(--line);border-radius:10px;padding:8px 10px;font-size:12px}
#legend i{display:inline-block;width:18px;height:0;border-top:3px solid;vertical-align:middle;margin-right:6px}#legend b{display:inline-block;width:10px;height:10px;border-radius:50%;margin:0 8px 0 4px;vertical-align:middle}
#controls{display:flex;gap:8px;align-items:center;padding:8px 10px;background:var(--panel);border-top:1px solid var(--line);flex-wrap:wrap}
button,select{background:#222b42;color:var(--txt);border:1px solid var(--line);border-radius:7px;padding:5px 10px;cursor:pointer;font:inherit}button:hover{background:#2c3757}
#seek{flex:1;min-width:160px}#tlabel{min-width:110px;text-align:right;font-variant-numeric:tabular-nums}
#side{width:370px;max-width:42%;overflow:auto;background:var(--panel);border-left:1px solid var(--line);padding:10px 12px}
@media(max-width:820px){#app{flex-direction:column}#side{width:100%;max-width:100%;max-height:45%;border-left:0;border-top:1px solid var(--line)}}
h3{margin:12px 0 5px;font-size:12px;letter-spacing:.06em;text-transform:uppercase;color:var(--mut)}h3:first-child{margin-top:0}
table{border-collapse:collapse;width:100%;font-size:12px;font-variant-numeric:tabular-nums}th,td{padding:3px 4px;border-bottom:1px solid var(--line);text-align:right}th:first-child,td:first-child,td.l,th.l{text-align:left}
tr.row{cursor:pointer}tr.row:hover{background:#222b42}tr.active{background:#1e3a4a!important;outline:1px solid var(--cy)}
.box{background:#121828;border:1px solid var(--line);border-radius:8px;padding:8px 10px;margin-bottom:6px}.mut{color:var(--mut)}
.ok{color:var(--gr)}.warn{color:var(--or)}.bad{color:var(--rd)}
.ev{padding:4px 6px;border-left:3px solid var(--or);margin:4px 0;background:#121828;cursor:pointer}.ev.hold{border-color:var(--pu)}.ev.abort{border-color:var(--rd)}.ev.resume{border-color:var(--gr)}
</style></head><body>
<div id="app">
 <div id="left">
  <div id="mapwrap">
   <div id="map"></div>
   <div id="maperr"><div><b>The 3D map could not start.</b><div id="maperrmsg" style="margin-top:6px"></div>
     <div class="mut" style="margin-top:8px">The route, distances, step-by-step explanation and playback controls still work &mdash; use the panel on the right. Check your internet connection (the map libraries load from unpkg.com) and reload.</div></div></div>
   <div id="hud"><span id="badge">-</span> <span id="hclock" class="mut"></span><div id="htext"></div><div class="kv" id="hkv"></div></div>
   <div id="legend"></div>
  </div>
  <div id="controls">
   <button id="bplay">&#9654; Play</button><button id="breplay">&#8635; Replay</button>
   <input id="seek" type="range" min="0" max="1000" value="0"><span id="tlabel"></span>
   <label>Speed <select id="speed"><option value="0.5">0.5&times;</option><option value="1" selected>1&times;</option><option value="2">2&times;</option><option value="5">5&times;</option><option value="10">10&times;</option></select></label>
   <button id="bfollow">Follow: off</button><button id="b3d">Top-down</button><button id="breset">Reset view</button>
  </div>
 </div>
 <div id="side">
  <h3>Current step</h3><div class="box" id="step"></div>
  <h3>Distances explained</h3><div class="box" id="dist"></div>
  <h3>Inspect (click a leg, waypoint, event or obstacle)</h3><div class="box" id="inspect"><span class="mut">Nothing selected yet.</span></div>
  <h3>Route legs &amp; waypoints</h3><div id="legs"></div>
  <h3>Events</h3><div id="events"></div>
  <h3>Data</h3><div class="box mut" id="notes"></div>
 </div>
</div>
<script type="application/json" id="sim-data">__DATA__</script>
<script>
var LIBS_FAILED = [];
function libFail(name) { LIBS_FAILED.push(name); }
</script>
<script src="__MAPLIBRE__" onerror="libFail('maplibre-gl 3.6.2')"></script>
<script src="__DECKGL__" onerror="libFail('deck.gl 8.9.35')"></script>
<script>
__CORE__
(function () {
'use strict';
const D = JSON.parse(document.getElementById('sim-data').textContent);
const meta = D.meta, frames = D.frames, M = D.metrics;
const T = frames[frames.length - 1].t;
const $ = id => document.getElementById(id);
const COL = { takeoff: '#4ade80', climbing: '#22d3ee', cruising: '#60a5fa', descending: '#38bdf8', landing: '#4ade80',
  holding: '#c084fc', rerouting: '#fb923c', landed: '#4ade80', aborted: '#f87171' };
const NAME = { takeoff: 'Takeoff', climbing: 'Climbing', cruising: 'Cruising', descending: 'Descending', landing: 'Landing',
  holding: 'Holding', rerouting: 'Re-routing', landed: 'Landed', aborted: 'Aborted' };
const RISKC = { clear: 'ok', caution: 'warn', critical: 'bad' };
const rgb = h => [parseInt(h.slice(1, 3), 16), parseInt(h.slice(3, 5), 16), parseInt(h.slice(5, 7), 16)];
const esc = s => String(s == null ? '' : s).replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const ll = (lon, lat) => lat.toFixed(6) + ', ' + lon.toFixed(6);

const S = { t: 0, playing: false, speed: 1, follow: false, topdown: false, sel: null, lastActive: '' };
let map = null, overlay = null, mapOK = false;

// ---------- static panels ----------
function distPanel(s) {
  const flown = s ? s.dist : 0, rem = s ? s.rem : M.planned_distance_m;
  $('dist').innerHTML =
    '<table><tr><td class="l">Planned route (3D)</td><td><b>' + fmtM(M.planned_distance_m) + '</b></td></tr>' +
    '<tr><td class="l">Straight-line (ground)</td><td><b>' + fmtM(M.straight_line_m) + '</b></td></tr>' +
    '<tr><td class="l">Actual flown (whole mission)</td><td><b>' + fmtM(M.actual_distance_m) + '</b></td></tr>' +
    '<tr><td class="l">Flown so far / remaining</td><td>' + fmtM(flown) + ' / ' + fmtM(rem) + '</td></tr>' +
    '<tr><td class="l">Detour factor (planned &divide; straight)</td><td>' + M.detour_factor.toFixed(3) + '&times;</td></tr>' +
    '<tr><td class="l">Actual &minus; planned</td><td>' + (M.actual_minus_planned_m >= 0 ? '+' : '') + fmtM(M.actual_minus_planned_m) + '</td></tr></table>' +
    '<div class="mut" style="margin-top:6px"><b>Planned</b> = length of the 3D route the planner chose (includes the climb and descent and any detours around buildings / no-fly zones). ' +
    '<b>Straight-line</b> = shortest ground distance start &rarr; destination, ignoring obstacles. ' +
    '<b>Actual</b> = distance really flown; it differs from planned only if a re-route happened (a hold adds time and energy, not distance). All values in metres.</div>';
}
function legsPanel() {
  let h = '';
  D.routes.forEach(r => {
    h += '<div class="mut" style="margin:6px 0 2px"><b>Route v' + r.ver + '</b> ' + (r.ver === 0 ? '(planned)' : '(re-route at t = ' + r.t_start.toFixed(0) + ' s)') +
      ' &middot; ' + fmtM(r.total_m) + '</div><table><tr><th class="l">Leg</th><th class="l">Phase</th><th>Horiz</th><th>Length</th><th>Cum.</th></tr>';
    r.legs.forEach(l => {
      h += '<tr class="row" id="leg-' + r.ver + '-' + l.idx + '" data-v="' + r.ver + '" data-l="' + l.idx + '"><td class="l">' + (l.idx + 1) + '</td><td class="l">' + NAME[l.phase] +
        '</td><td>' + l.horiz_m.toFixed(1) + '</td><td>' + l.length_m.toFixed(1) + '</td><td>' + l.cum_end_takeoff_m.toFixed(1) + '</td></tr>';
    });
    h += '</table><div class="mut" style="font-size:11px">Horiz / Length / Cum. in metres; Cum. = distance from takeoff at the end of the leg.</div>';
  });
  $('legs').innerHTML = h;
  document.querySelectorAll('tr.row').forEach(tr => tr.onclick = () => selectLeg(+tr.dataset.v, +tr.dataset.l, true));
}
function eventsPanel() {
  if (!D.events.length) { $('events').innerHTML = '<span class="mut">No re-route or hold events in this flight.</span>'; return; }
  $('events').innerHTML = D.events.map(e => '<div class="ev ' + e.type + '" data-e="' + e.id + '"><b>' + esc(e.type.toUpperCase()) + '</b> at t = ' + e.t.toFixed(1) + ' s, ' +
    fmtM(e.cum_m) + ' from takeoff, ' + fmtM(e.alt_m) + ' AGL<br>' + esc(e.reason) + '<br><span class="mut">' + esc(e.detail) + '</span></div>').join('');
  document.querySelectorAll('.ev').forEach(d => d.onclick = () => selectEvent(d.dataset.e, true));
}
function notesPanel() {
  const c = D.camera;
  $('notes').innerHTML = esc(D.buildings_note || D.buildings_source) + '<br>Buildings: ' + D.buildings.length + ' &middot; No-fly zones: ' + D.nofly.length +
    ' &middot; Moving obstacles: ' + D.dynamic.length + '<br>Frame: local metres from the start point (x east, y north, z up = metres above ground). Heights and altitudes share the same ground level.' +
    (D.warnings && D.warnings.filter(w => w !== D.buildings_note).length ? '<br><span class="warn">' + D.warnings.filter(w => w !== D.buildings_note).map(esc).join('<br>') + '</span>' : '') +
    (D.failure ? '<br><span class="bad">' + esc(D.failure) + '</span>' : '');
}
function legendPanel() {
  $('legend').innerHTML =
    '<div><i style="border-color:#22d3ee"></i>Planned route</div><div><i style="border-color:#4ade80"></i>Actual flown path</div>' +
    '<div><i style="border-color:#fb923c"></i>Re-route</div><div><i style="border-color:#888"></i>Straight line (ground)</div>' +
    '<div><b style="background:#fb923c"></b>Re-route event <b style="background:#c084fc"></b>Hold event</div>' +
    '<div><b style="background:#f87171"></b>No-fly zone / moving obstacle <b style="background:#facc15"></b>Drone</div>';
}

// ---------- selection ----------
function legDetail(r, l) {
  const w0 = r.waypoints[l.a], w1 = r.waypoints[l.b];
  return '<b>Route v' + r.ver + ' &middot; leg ' + (l.idx + 1) + ' of ' + r.legs.length + ' &middot; ' + NAME[l.phase] + '</b><table>' +
    '<tr><td class="l">From</td><td>' + esc(w0.label) + ' &middot; ' + ll(l.lon0, l.lat0) + ' &middot; ' + fmtM(l.z0) + ' AGL</td></tr>' +
    '<tr><td class="l">To</td><td>' + esc(w1.label) + ' &middot; ' + ll(l.lon1, l.lat1) + ' &middot; ' + fmtM(l.z1) + ' AGL</td></tr>' +
    '<tr><td class="l">Segment length (3D)</td><td><b>' + fmtM(l.length_m) + '</b> = &radic;(' + l.horiz_m.toFixed(1) + '&sup2; + ' + l.vert_m.toFixed(1) + '&sup2;)</td></tr>' +
    '<tr><td class="l">Horizontal / vertical</td><td>' + fmtM(l.horiz_m) + ' / ' + (l.vert_m >= 0 ? '+' : '') + fmtM(l.vert_m) + '</td></tr>' +
    '<tr><td class="l">Cumulative from takeoff</td><td>' + fmtM(l.cum_start_takeoff_m) + ' &rarr; <b>' + fmtM(l.cum_end_takeoff_m) + '</b></td></tr>' +
    '<tr><td class="l">Speed / time / energy</td><td>' + l.speed_ms.toFixed(1) + ' m/s / ' + l.time_s.toFixed(1) + ' s / ' + l.energy_wh.toFixed(2) + ' Wh</td></tr>' +
    '<tr><td class="l">Closest obstacle</td><td>' + (l.clearance_obj ? esc(l.clearance_obj) + ' at ' + fmtM(l.min_clearance_m) : 'none within 100 m') + '</td></tr></table>' +
    '<div class="mut" style="margin-top:4px">' + esc(l.why) + '</div>';
}
function wpDetail(r, w) {
  return '<b>' + esc(w.label) + '</b> (route v' + r.ver + ')<table>' +
    '<tr><td class="l">Latitude, longitude</td><td>' + ll(w.lon, w.lat) + '</td></tr>' +
    '<tr><td class="l">Local x, y (east, north)</td><td>' + w.x.toFixed(1) + ', ' + w.y.toFixed(1) + ' m</td></tr>' +
    '<tr><td class="l">Altitude (AGL)</td><td>' + fmtM(w.z) + '</td></tr>' +
    '<tr><td class="l">Cumulative from takeoff</td><td><b>' + fmtM(w.cum_takeoff_m) + '</b></td></tr>' +
    '<tr><td class="l">Closest obstacle</td><td>' + (w.clearance_obj ? esc(w.clearance_obj) + ' at ' + fmtM(w.clearance_m) : 'none within 100 m') + '</td></tr></table>' +
    '<div class="mut" style="margin-top:4px">' + esc(w.why) + '</div>';
}
function evDetail(e) {
  return '<b>' + esc(e.type.toUpperCase()) + ' event ' + e.id + '</b><table>' +
    '<tr><td class="l">Time</td><td>' + e.t.toFixed(1) + ' s</td></tr><tr><td class="l">Location</td><td>' + ll(e.lon, e.lat) + '</td></tr>' +
    '<tr><td class="l">Altitude (AGL)</td><td>' + fmtM(e.alt_m) + '</td></tr><tr><td class="l">Distance flown so far</td><td>' + fmtM(e.cum_m) + '</td></tr>' +
    (e.obstacle ? '<tr><td class="l">Obstacle</td><td>' + esc(e.obstacle) + '</td></tr>' : '') + '</table>' +
    '<div style="margin-top:4px">' + esc(e.reason) + '</div><div class="mut">' + esc(e.detail) + '</div>';
}
function setInspect(h) { $('inspect').innerHTML = h; }
function selectLeg(v, i, fly) { const r = D.routes[v], l = r.legs[i]; S.sel = { k: 'leg', v: v, i: i }; setInspect(legDetail(r, l)); if (fly) focus((l.lon0 + l.lon1) / 2, (l.lat0 + l.lat1) / 2); render(true); }
function selectWp(v, i, fly) { const r = D.routes[v], w = r.waypoints[i]; S.sel = { k: 'wp', v: v, i: i }; setInspect(wpDetail(r, w)); if (fly) focus(w.lon, w.lat); render(true); }
function selectEvent(id, seek) { const e = D.events.find(x => x.id === id); if (!e) return; S.sel = { k: 'ev', id: id }; setInspect(evDetail(e)); if (seek) { S.t = Math.min(T, e.t); } focus(e.lon, e.lat); render(true); }
function focus(lon, lat) { if (mapOK) map.easeTo({ center: [lon, lat], duration: 400 }); }
function pickInfo(info) {
  if (!info || !info.object) return;
  const id = info.layer.id, o = info.object;
  if (id === 'legs') selectLeg(o.v, o.i, false);
  else if (id === 'wps') selectWp(o.v, o.i, false);
  else if (id === 'events') selectEvent(o.id, false);
  else if (id === 'buildings') setInspect('<b>' + esc(o.name) + '</b><div class="mut">Height ' + fmtM(o.height) + (o.assumed ? ' (assumed &mdash; no height in the map data)' : '') + ' above ground. Source: ' + esc(o.src) + '.</div>');
  else if (id === 'nofly') setInspect('<b>' + esc(o.name) + '</b><div class="mut">Radius ' + fmtM(o.radius) + ' &middot; centre ' + ll(o.lon, o.lat) + ' &middot; closed from ground to ' + fmtM(o.top) + ' AGL. The route must stay out of it.</div>');
  else if (id === 'obst') setInspect('<b>' + esc(o.name) + '</b><div class="mut">Moving obstacle, radius ' + fmtM(o.radius) + ', altitude ' + fmtM(o.p0[2]) + ' AGL, speed ' + Math.hypot(o.v[0], o.v[1]).toFixed(1) + ' m/s. It is detected inside the detection range and triggers a re-route or hold if it would pass too close.</div>');
}
function tip(info) {
  if (!info || !info.object) return null;
  const o = info.object, id = info.layer.id;
  let t = null;
  if (id === 'legs') { const l = D.routes[o.v].legs[o.i]; t = 'Leg ' + (l.idx + 1) + ' (v' + o.v + ') ' + NAME[l.phase] + '\nlength ' + fmtM(l.length_m) + ' | cum. ' + fmtM(l.cum_end_takeoff_m) + '\nclick for details'; }
  else if (id === 'wps') { const w = D.routes[o.v].waypoints[o.i]; t = w.label + '\n' + fmtM(w.z) + ' AGL | ' + fmtM(w.cum_takeoff_m) + ' from takeoff'; }
  else if (id === 'events') t = o.type.toUpperCase() + ' at t=' + o.t.toFixed(0) + ' s\n' + o.reason;
  else if (id === 'buildings') t = o.name;
  else if (id === 'nofly') t = o.name + ' (r ' + fmtM(o.radius) + ')';
  else if (id === 'obst') t = o.name;
  return t ? { text: t, style: { backgroundColor: '#111a', color: '#fff', fontSize: '12px', whiteSpace: 'pre' } } : null;
}

// ---------- map ----------
const shadowR = 7;
const legData = [], wpData = [];
D.routes.forEach(r => {
  r.legs.forEach(l => legData.push({ v: r.ver, i: l.idx, path: [[l.lon0, l.lat0, l.z0], [l.lon1, l.lat1, l.z1]] }));
  r.waypoints.forEach(w => wpData.push({ v: r.ver, i: w.idx, p: [w.lon, w.lat, w.z], kind: w.kind }));
});
const groundTrack = D.routes[0].waypoints.map(w => [w.lon, w.lat, 0.2]);
const staticLayers = () => {
  const L = deck;
  return [
    new L.PolygonLayer({ id: 'buildings', data: D.buildings, getPolygon: d => d.polygon, extruded: true, getElevation: d => d.height,
      getFillColor: d => d.height > 60 ? [196, 120, 120, 215] : d.height > 25 ? [150, 160, 190, 215] : [130, 150, 175, 215], getLineColor: [60, 70, 90], pickable: true, autoHighlight: true }),
    new L.PolygonLayer({ id: 'nofly', data: D.nofly, getPolygon: d => d.polygon, extruded: true, getElevation: d => d.top, getFillColor: [248, 113, 113, 70], getLineColor: [248, 113, 113], pickable: true }),
    new L.PathLayer({ id: 'straight', data: [D.paths.straight], getPath: d => d, getColor: [150, 150, 150], widthMinPixels: 2 }),
    new L.PathLayer({ id: 'ground', data: [groundTrack], getPath: d => d, getColor: [34, 211, 238, 110], widthMinPixels: 2 }),
    new L.PathLayer({ id: 'obst-trace', data: D.traces.obstacles, getPath: d => d.path, getColor: [248, 113, 113, 120], widthMinPixels: 1 }),
  ];
};
let STATIC = null;
function dynamicLayers(s) {
  const L = deck, g = droneGeometry(s, meta, shadowR);
  const sel = S.sel;
  const selLeg = sel && sel.k === 'leg' ? legData.filter(d => d.v === sel.v && d.i === sel.i) : [];
  const selWp = sel && sel.k === 'wp' ? wpData.filter(d => d.v === sel.v && d.i === sel.i) : [];
  const upto = frames.slice(0, s.i + 1).map(f => [f.lon, f.lat, f.z]); upto.push([s.lon, s.lat, s.z]);
  return [
    new L.PathLayer({ id: 'legs', data: legData, getPath: d => d.path, getColor: d => d.v === 0 ? [34, 211, 238] : [251, 146, 60], widthMinPixels: 4, pickable: true, capRounded: true }),
    new L.PathLayer({ id: 'legsel', data: selLeg, getPath: d => d.path, getColor: [250, 204, 21], widthMinPixels: 8, capRounded: true }),
    new L.PathLayer({ id: 'flown', data: [upto], getPath: d => d, getColor: [74, 222, 128], widthMinPixels: 5, capRounded: true }),
    new L.ScatterplotLayer({ id: 'wps', data: wpData, getPosition: d => d.p, getRadius: 5, radiusUnits: 'meters', radiusMinPixels: 5, radiusMaxPixels: 9, pickable: true,
      getFillColor: d => d.kind === 'start' ? [74, 222, 128] : d.kind === 'destination' ? [248, 113, 113] : d.v === 0 ? [34, 211, 238] : [251, 146, 60], getLineColor: [255, 255, 255], stroked: true, lineWidthMinPixels: 1 }),
    new L.ScatterplotLayer({ id: 'wpsel', data: selWp, getPosition: d => d.p, getRadius: 9, radiusUnits: 'meters', radiusMinPixels: 11, getFillColor: [250, 204, 21, 200] }),
    new L.ScatterplotLayer({ id: 'events', data: D.events.filter(e => e.type === 'reroute' || e.type === 'hold' || e.type === 'abort'), getPosition: e => [e.lon, e.lat, e.alt_m],
      getRadius: 8, radiusUnits: 'meters', radiusMinPixels: 10, pickable: true, stroked: true, lineWidthMinPixels: 2, getLineColor: [255, 255, 255],
      getFillColor: e => e.type === 'hold' ? [192, 132, 252] : e.type === 'abort' ? [248, 113, 113] : [251, 146, 60] }),
    new L.ScatterplotLayer({ id: 'obst', data: D.dynamic.map(o => Object.assign({}, o, { p: obstaclePos(o, s.t, meta) })), getPosition: o => o.p, getRadius: o => o.radius, radiusUnits: 'meters',
      radiusMinPixels: 6, getFillColor: [248, 113, 113, 120], stroked: true, getLineColor: [248, 113, 113], lineWidthMinPixels: 2, pickable: true, updateTriggers: { getPosition: [s.t] } }),
    new L.PolygonLayer({ id: 'shadow', data: [g.shadow], getPolygon: d => d, getFillColor: [0, 0, 0, 150], stroked: false, updateTriggers: { getPolygon: [s.lon, s.lat] } }),
    new L.LineLayer({ id: 'drop', data: [g.drop], getSourcePosition: d => d[0], getTargetPosition: d => d[1], getColor: [250, 204, 21, 220], getWidth: 2, widthUnits: 'pixels' }),
    new L.ScatterplotLayer({ id: 'drone', data: [g.pos], getPosition: d => d, getRadius: 5, radiusUnits: 'meters', radiusMinPixels: 8, getFillColor: [250, 204, 21], stroked: true, getLineColor: [0, 0, 0], lineWidthMinPixels: 2,
      updateTriggers: { getPosition: [s.lon, s.lat, s.z] } }),
  ];
}
function initMap() {
  if (LIBS_FAILED.length || typeof maplibregl === 'undefined' || typeof deck === 'undefined') {
    const miss = LIBS_FAILED.length ? LIBS_FAILED.join(', ') : (typeof maplibregl === 'undefined' ? 'maplibre-gl 3.6.2' : 'deck.gl 8.9.35');
    $('maperrmsg').textContent = 'Could not load: ' + miss + '.';
    $('maperr').style.display = 'flex';
    return;
  }
  try {
    const c = D.camera;
    map = new maplibregl.Map({ container: 'map', center: [c.lon, c.lat], zoom: c.zoom, pitch: c.pitch, bearing: c.bearing, antialias: true,
      style: { version: 8, sources: { base: { type: 'raster', tileSize: 256, attribution: '&copy; OpenStreetMap contributors &copy; CARTO',
        tiles: ['a', 'b', 'c'].map(s => 'https://' + s + '.basemaps.cartocdn.com/light_all/{z}/{x}/{y}.png') } },
        layers: [{ id: 'bg', type: 'background', paint: { 'background-color': '#dfe6ee' } }, { id: 'base', type: 'raster', source: 'base' }] } });
    map.addControl(new maplibregl.NavigationControl({ visualizePitch: true }), 'top-right');
    overlay = new deck.MapboxOverlay({ interleaved: false, layers: [], onClick: pickInfo, getTooltip: tip });
    map.addControl(overlay);
    STATIC = staticLayers();
    mapOK = true;
  } catch (err) {
    mapOK = false;
    $('maperrmsg').textContent = 'Your browser could not start WebGL (' + err.message + ').';
    $('maperr').style.display = 'flex';
  }
}

// ---------- render ----------
function render(force) {
  const s = sampleAt(frames, S.t);
  const a = s.a, st = a.state;
  const bd = $('badge'); bd.textContent = NAME[st] || st; bd.style.background = COL[st] || '#334'; bd.style.color = '#001';
  $('hclock').textContent = 't = ' + s.t.toFixed(1) + ' s / ' + T.toFixed(1) + ' s';
  $('htext').textContent = a.text;
  const clrTxt = (a.clr_obj ? fmtM(s.clr) + ' to ' + a.clr_obj : 'no obstacle within 100 m');
  const kv = [
    ['Position', ll(s.lon, s.lat)], ['Altitude (AGL)', fmtM(s.z)], ['Travelled', fmtM(s.dist)], ['Remaining', fmtM(s.rem)],
    ['Energy used / left', s.e_used.toFixed(2) + ' / ' + s.e_left.toFixed(2) + ' Wh (' + s.batt.toFixed(0) + '%)'],
    ['Clearance', clrTxt + ' &middot; <span class="' + RISKC[a.risk] + '">' + a.risk + '</span>']];
  $('hkv').innerHTML = kv.map(r => '<span>' + r[0] + '</span><span>' + r[1] + '</span>').join('');
  const r = D.routes[a.ver], l = r.legs[Math.min(a.leg, r.legs.length - 1)];
  $('step').innerHTML = '<b>' + (NAME[st] || st) + '</b> &middot; route v' + a.ver + ', leg ' + (a.leg + 1) + ' of ' + r.legs.length + ' (' + fmtM(l.length_m) + ' long, cumulative ' + fmtM(l.cum_start_takeoff_m) + ' &rarr; ' + fmtM(l.cum_end_takeoff_m) + ')' +
    '<div style="margin:4px 0">' + esc(a.text) + '</div><table>' +
    '<tr><td class="l">Position</td><td>' + ll(s.lon, s.lat) + '</td></tr><tr><td class="l">Altitude (AGL)</td><td>' + fmtM(s.z) + '</td></tr>' +
    '<tr><td class="l">Distance travelled</td><td>' + fmtM(s.dist) + '</td></tr><tr><td class="l">Distance remaining</td><td>' + fmtM(s.rem) + '</td></tr>' +
    '<tr><td class="l">Energy used / remaining</td><td>' + s.e_used.toFixed(2) + ' Wh / ' + s.e_left.toFixed(2) + ' Wh (' + s.batt.toFixed(0) + '%)</td></tr>' +
    '<tr><td class="l">Obstacle clearance</td><td>' + clrTxt + ' &middot; <span class="' + RISKC[a.risk] + '">' + a.risk + '</span></td></tr></table>' +
    (a.ev ? '<div class="ev ' + (D.events.find(e => e.id === a.ev) || {}).type + '" style="cursor:default">Event ' + a.ev + ': ' + esc((D.events.find(e => e.id === a.ev) || {}).reason) + '</div>' : '');
  distPanel(s);
  const key = a.ver + '-' + a.leg;
  if (key !== S.lastActive) {
    const old = document.getElementById('leg-' + S.lastActive); if (old) old.classList.remove('active');
    const cur = document.getElementById('leg-' + key); if (cur) { cur.classList.add('active'); }
    S.lastActive = key;
  }
  $('seek').value = T > 0 ? Math.round(1000 * S.t / T) : 0;
  $('tlabel').textContent = s.t.toFixed(1) + ' / ' + T.toFixed(1) + ' s';
  if (mapOK) {
    overlay.setProps({ layers: STATIC.concat(dynamicLayers(s)) });
    if (S.follow) map.jumpTo({ center: [s.lon, s.lat] });
  }
}

// ---------- playback ----------
let last = null;
function tick(now) {
  if (S.playing) {
    if (last !== null) S.t = Math.min(T, S.t + (now - last) / 1000 * S.speed);
    last = now;
    if (S.t >= T) { S.playing = false; $('bplay').innerHTML = '&#9654; Play'; }
    render();
    requestAnimationFrame(tick);
  } else { last = null; }
}
function play() { if (S.t >= T) S.t = 0; S.playing = true; $('bplay').innerHTML = '&#10074;&#10074; Pause'; last = null; requestAnimationFrame(tick); }
function pause() { S.playing = false; $('bplay').innerHTML = '&#9654; Play'; render(); }
$('bplay').onclick = () => S.playing ? pause() : play();
$('breplay').onclick = () => { S.t = 0; play(); };
$('seek').oninput = e => { S.t = T * (+e.target.value) / 1000; render(); };
$('speed').onchange = e => { S.speed = +e.target.value; };
$('bfollow').onclick = () => { S.follow = !S.follow; $('bfollow').textContent = 'Follow: ' + (S.follow ? 'on' : 'off'); render(); };
$('b3d').onclick = () => { if (!mapOK) return; S.topdown = !S.topdown; map.easeTo({ pitch: S.topdown ? 0 : D.camera.pitch, duration: 500 }); $('b3d').textContent = S.topdown ? '3D view' : 'Top-down'; };
$('breset').onclick = () => { if (!mapOK) return; S.follow = false; $('bfollow').textContent = 'Follow: off'; S.topdown = false; $('b3d').textContent = 'Top-down';
  map.easeTo({ center: [D.camera.lon, D.camera.lat], zoom: D.camera.zoom, pitch: D.camera.pitch, bearing: D.camera.bearing, duration: 600 }); };

legsPanel(); eventsPanel(); notesPanel(); legendPanel();
initMap();
render(true);
window.__sim = { sampleAt: s => sampleAt(frames, s), render: render, state: S };
})();
</script></body></html>
"""


def build_viewer_html(result: dict) -> str:
    """Return the full viewer as one self-contained HTML string."""
    html = HTML.replace("__MAPLIBRE_CSS__", MAPLIBRE_CSS).replace("__MAPLIBRE__", MAPLIBRE)
    html = html.replace("__DECKGL__", DECKGL).replace("__CORE__", CORE_JS)
    return html.replace("__DATA__", safe_json(payload(result)))


# --------------------------------------------------------------------------
def _selftest():
    import copy
    import os
    import shutil
    import subprocess
    import tempfile

    import sim3d

    ok = True

    def check(name, cond, extra=""):
        nonlocal ok
        print(("PASS " if cond else "FAIL ") + name + (f"  {extra}" if extra else ""))
        ok = ok and cond

    res = sim3d.run_simulation(sim3d.Config(use_osm=False))
    check("simulation for viewer test", res.get("ok"), str(res.get("error")))
    evil = copy.deepcopy(res)
    evil["buildings"][0]["name"] = '</script><script>alert(1)</script> & <!-- "x" \u2028'
    html = build_viewer_html(evil)
    body = re.search(r'<script type="application/json" id="sim-data">(.*?)</script>', html, re.S).group(1)
    check("embedded JSON has no raw < > & or U+2028", not re.search(r"[<>&\u2028\u2029]", body))
    check("embedded JSON round-trips", json.loads(body)["buildings"][0]["name"] == evil["buildings"][0]["name"])
    check("exactly one </script> per script block", html.count("</script>") == html.count("<script"))
    check("libraries pinned", "maplibre-gl@3.6.2" in html and "deck.gl@8.9.35" in html)
    check("friendly load error present", "could not start" in html and "libFail" in html)

    node = shutil.which("node")
    if not node:
        print("SKIP JS interpolation checks (node not found)")
        return 0 if ok else 1
    core = re.search(r"// <CORE>.*?// </CORE>", html, re.S).group(0)
    data = json.loads(body)
    harness = core + r"""
const D = JSON.parse(require('fs').readFileSync(process.argv[2], 'utf8'));
const F = D.frames, meta = D.meta; let bad = [];
const dot = (a, b) => a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
function distToRoute(ver, p) {                      // metres from p (local x,y,z) to route polyline
  let best = 1e18;
  for (const l of D.routes[ver].legs) {
    const a = [l.x0, l.y0, l.z0], u = [l.x1-l.x0, l.y1-l.y0, l.z1-l.z0], w = [p[0]-a[0], p[1]-a[1], p[2]-a[2]];
    const t = Math.max(0, Math.min(1, dot(w, u) / dot(u, u)));
    best = Math.min(best, Math.hypot(w[0]-t*u[0], w[1]-t*u[1], w[2]-t*u[2]));
  }
  return best;
}
const T = F[F.length-1].t; let n = 0, maxOff = 0, maxShadow = 0, maxDist = 0;
for (let k = 0; k <= 4000; k++) {
  const t = T * k / 4000, s = sampleAt(F, t); n++;
  const loc = geoToLocal(meta, s.lon, s.lat);
  if (!(s.z >= -1e-9)) bad.push('negative altitude at t=' + t);
  const off = distToRoute(s.a.ver, [loc[0], loc[1], s.z]); maxOff = Math.max(maxOff, off);
  if (s.a.state !== 'holding' && s.a.state !== 'aborted' && off > 1e-3) bad.push('off route ' + off + ' at t=' + t);
  const g = droneGeometry(s, meta, 7);
  if (g.drop[0][0] !== g.pos[0] || g.drop[0][1] !== g.pos[1] || g.drop[0][2] !== 0 || g.drop[1][2] !== s.z) bad.push('drop line misaligned');
  let cx = 0, cy = 0; const ring = g.shadow.slice(0, -1);
  ring.forEach(p => { cx += p[0]; cy += p[1]; }); cx /= ring.length; cy /= ring.length;
  const dm = Math.hypot((cx - s.lon) * meta.m_per_deg_lon, (cy - s.lat) * meta.m_per_deg_lat); maxShadow = Math.max(maxShadow, dm);
  const r0 = geoToLocal(meta, ring[0][0], ring[0][1]); const rr = Math.hypot(r0[0]-loc[0], r0[1]-loc[1]);
  if (Math.abs(rr - 7) > 1e-6) bad.push('shadow radius ' + rr);
  // distance travelled must equal integral of speed along the path between frames
  const aa = s.a, bb = s.b;
  const seg = Math.hypot(bb.x-aa.x, bb.y-aa.y, bb.z-aa.z);
  const expect = aa.dist + seg * s.u;
  maxDist = Math.max(maxDist, Math.abs(expect - s.dist));
}
if (maxShadow > 1e-6) bad.push('shadow centre offset ' + maxShadow);
if (maxDist > 1e-6 && !D.events.length) bad.push('distance not linear ' + maxDist);
// frame lon/lat/z match local x,y
for (const f of F) { const l = localToGeo(meta, f.x, f.y); if (Math.abs(l[0]-f.lon) > 1e-9 || Math.abs(l[1]-f.lat) > 1e-9) { bad.push('frame lon/lat mismatch'); break; } }
// seek edge cases
for (const t of [-5, 0, T, T + 100]) { const s = sampleAt(F, t); if (!isFinite(s.lon) || !isFinite(s.dist)) bad.push('seek edge ' + t); }
// duplicate-time frames (events) resolve to the later frame
for (let i = 0; i + 1 < F.length; i++) if (F[i].t === F[i+1].t && sampleAt(F, F[i].t).i < i) { bad.push('dup-time frame skipped at ' + i); break; }
console.log(JSON.stringify({ samples: n, maxOffRouteM: maxOff, maxShadowOffsetM: maxShadow, maxDistErrM: maxDist, bad: bad.slice(0, 5) }));
process.exit(bad.length ? 1 : 0);
"""
    with tempfile.TemporaryDirectory() as td:
        jp, hp = os.path.join(td, "d.json"), os.path.join(td, "h.js")
        with open(jp, "w") as f:
            f.write(json.dumps(data))
        with open(hp, "w") as f:
            f.write(harness)
        for name, cfg in (("reroute run", sim3d.Config(use_osm=False)),
                          ("hold run", sim3d.Config(use_osm=False, response="hold", wind_east_ms=4))):
            r = sim3d.run_simulation(cfg)
            with open(jp, "w") as f:
                f.write(json.dumps(payload(r)))
            p = subprocess.run([node, hp, jp], capture_output=True, text=True)
            check(f"JS interpolation core ({name})", p.returncode == 0, (p.stdout or p.stderr).strip()[:300])
    print("ALL CHECKS PASSED" if ok else "SOME CHECKS FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        raise SystemExit(_selftest())
    print(__doc__)
