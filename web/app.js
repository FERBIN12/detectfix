import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { Line2 } from "three/addons/lines/Line2.js";
import { LineMaterial } from "three/addons/lines/LineMaterial.js";
import { LineGeometry } from "three/addons/lines/LineGeometry.js";

// ---- helpers ---------------------------------------------------------------
const $ = (s) => document.querySelector(s);
async function getJSON(u) { const r = await fetch(u); if (!r.ok) throw new Error(`${u}: ${r.status}`); return r.json(); }
const SEV_COLOR = { severe: 0xff5a5a, poor: 0xff9d42, fair: 0xffd24a, good: 0x4ade80 };
const fmt = (n) => (n == null ? "—" : Number(n).toLocaleString());

async function main() {
  let scene, kpis, coop, result, frames, triage;
  try {
    [scene, kpis, coop, result, frames, triage] = await Promise.all([
      getJSON("/web/data/scene.json"), getJSON("/web/data/kpis.json"),
      getJSON("/web/data/coop.json"), getJSON("/web/data/result.json"),
      getJSON("/web/data/frames.json"), getJSON("/web/data/triage.json"),
    ]);
  } catch (e) {
    $("#mission-id").textContent = "no data — run export_web.py first";
    console.error(e); return;
  }
  renderHeader(scene, result);
  renderKPIs(kpis);
  renderTimeline(coop);
  renderDetections(result.detections);
  renderReport(result.report);
  renderStatusControl();
  renderTriage(triage);
  initFeeds(frames);
  init3D(scene);
}

// ---- 1. header -------------------------------------------------------------
function renderHeader(scene, result) {
  $("#mission-id").textContent = scene.mission_id || "";
  const d = result.dashboard || {};
  $("#headline").textContent = d.headline || "";
}

// ---- 2. KPI tiles ----------------------------------------------------------
function renderKPIs(k) {
  const tiles = [
    { v: k.model_defects, k: "model detections", cls: "" },
    { v: `${k.gt_total}`, sub: `${k.gt_above}↑ / ${k.gt_below}↓`, k: "ground-truth defects", cls: "violet" },
    { v: `${k.flagged}/${k.rov_inspected}`, k: "flagged → ROV inspected", cls: "good" },
    { v: k.nav_err_m, sub: "m", k: "ROV nav error vs truth", cls: "amber" },
    { v: k.rov_distance_m, sub: "m", k: "ROV distance travelled", cls: "amber" },
    { v: fmt(k.lidar_points), k: "LiDAR points (above)", cls: "" },
    { v: fmt(k.sonar_points), k: "sonar points (below)", cls: "amber" },
    { v: k.mission_s, sub: "s", k: "mission duration", cls: "" },
    { v: k.drone_waypoints, k: "drone waypoints", cls: "violet" },
  ];
  $("#kpis").innerHTML = tiles.map((t) =>
    `<div class="kpi ${t.cls}"><div class="v">${t.v ?? "—"}${t.sub ? ` <small>${t.sub}</small>` : ""}</div><div class="k">${t.k}</div></div>`
  ).join("");
}

// ---- 5. cooperation timeline ----------------------------------------------
function renderTimeline(coop) {
  const host = $("#timeline");
  const span = coop.span_s || 1;
  const log = coop.log || {};
  $("#coop-summary").textContent =
    `${(log.flagged || []).length} defects flagged at the water line · ${(log.done || []).length} inspected by ROV · ${coop.span_s}s mission`;

  // axis ticks every ~60 s
  let html = `<div class="axis"></div>`;
  for (let t = 0; t <= span; t += 60) {
    html += `<div class="tick" style="left:${(t / span) * 100}%">${t}s</div>`;
  }
  // events: classify by text; stagger captions on two rows when close in time
  const evs = (coop.events || []).slice().sort((a, b) => a.t - b.t);
  let lastT = -999, row = 0;
  for (const e of evs) {
    const pct = (e.t / span) * 100;
    let cls = "drone", cap = e.text;
    if (/diving/i.test(e.text)) { cls = "dive"; cap = e.text.replace(/.*saw /, "").replace(" -> ROV diving", " ↓"); }
    else if (/inspected|done/i.test(e.text)) { cls = "done"; cap = e.text.replace(/.*saw /, "").replace(" -> ROV inspected", " ✓"); }
    else { cap = e.text.replace(/.*saw /, "").replace(/ ->.*/, ""); }
    // alternate caption row when this event is within 15 s of the previous one
    if (e.t - lastT < 15) { row = row ? 0 : 1; } else { row = 0; }
    lastT = e.t;
    html += `<div class="ev ${cls} row${row}" style="left:${pct}%" title="${e.t}s · ${e.text}">
      <div class="dot"></div><div class="cap">${cap}</div></div>`;
  }
  host.innerHTML = html;
}

// ---- 6. detections + report -----------------------------------------------
function renderDetections(dets) {
  $("#det-count").textContent = `${dets.length} detected`;
  const tb = $("#det-table tbody");
  tb.innerHTML = dets.map((d) => {
    const s = d.location.structure;
    const ev = (d.evidence_frame_ids || []).join(", ");
    return `<tr>
      <td>${s.element_id || "—"}</td>
      <td>${d.class}</td>
      <td class="sev ${d.severity}">${d.severity}</td>
      <td class="mono">${(d.confidence * 100).toFixed(0)}%</td>
      <td class="mono">${ev || "—"}</td>
    </tr>`;
  }).join("");
}

function renderReport(report) {
  const el = $("#report");
  if (!report) { el.textContent = "—"; return; }
  el.innerHTML = (report.narrative || "").replace(/\[(D-[0-9a-f]+)\]/g, '<span class="cite">[$1]</span>');
  const isLLM = report.source === "llm";
  $("#report-badge").textContent = isLLM
    ? `Claude · ${report.backend}` : `deterministic (no ${report.backend} key)`;
}

function renderStatusControl() {
  const ctl = $("#status-ctl");
  ctl.querySelectorAll("button").forEach((b) => {
    b.onclick = () => {
      ctl.querySelectorAll("button").forEach((x) => x.classList.remove("on"));
      b.classList.add("on");
    };
  });
}

// ---- 7. statewide triage ---------------------------------------------------
function renderTriage(t) {
  const s = t.summary || {};
  const stats = [
    { v: fmt(s.total), k: "bridges statewide", cls: "" },
    { v: fmt(s.over_water), k: "over water", cls: "" },
    { v: fmt(s.poor), k: "in Poor condition", cls: "warn" },
    { v: fmt(s.underwater_required), k: "need underwater inspection", cls: "warn" },
    { v: fmt(s.scour_critical), k: "scour critical", cls: "alert" },
  ];
  $("#triage-stats").innerHTML = stats.map((x) =>
    `<div class="tstat ${x.cls}"><div class="v">${x.v}</div><div class="k">${x.k}</div></div>`).join("");

  const tb = $("#triage-table tbody");
  tb.innerHTML = (t.top || []).map((b, i) => {
    const uw = b.underwater || {};
    const overdue = uw.overdue ? `<span class="pill bad">${uw.months_overdue} mo</span>` : `<span class="pill">on schedule</span>`;
    const scour = b.scour_code != null && b.scour_code !== "" ? `<span class="pill ${(+b.scour_code <= 3) ? "bad" : "warn"}">${b.scour_code}</span>` : "—";
    return `<tr>
      <td class="mono">${i + 1}</td>
      <td class="mono">${b.structure_number}</td>
      <td>${(b.facility_carried || "").trim()} <small>/ ${(b.features_desc || "").trim()}</small></td>
      <td class="mono">${b.score}</td>
      <td class="mono">${b.lowest_condition ?? "—"}/9</td>
      <td>${scour}</td>
      <td>${overdue}</td>
      <td class="reasons">${(b.reasons || []).join(" · ")}</td>
    </tr>`;
  }).join("");
}

// ---- 4. sensor feeds -------------------------------------------------------
function initFeeds(manifest) {
  const order = ["rgb", "thermal", "rov_cam", "sonar"];
  const tabs = $("#feed-tabs"), img = $("#feed-img"), scrub = $("#f-scrub"), label = $("#f-label");
  const streams = order.filter((k) => manifest[k] && manifest[k].frames.length);
  if (!streams.length) { label.textContent = "no frames"; return; }

  let cur = streams[0], i = 0, playing = false, timer = null;
  tabs.innerHTML = streams.map((k) =>
    `<button data-k="${k}" class="${k === cur ? "on" : ""}">${manifest[k].label}</button>`).join("");

  function srcFor(k, idx) {
    // Drone RGB prefers the annotated (model-overlay) still when one exists.
    const f = manifest[k].frames[idx];
    return `/web/data/${k}/${f.name}`;
  }
  function show(idx) {
    const fr = manifest[cur].frames;
    i = (idx + fr.length) % fr.length;
    img.src = srcFor(cur, i);
    scrub.value = i;
    label.textContent = `${i + 1}/${fr.length}`;
  }
  function selectStream(k) {
    cur = k; i = 0;
    tabs.querySelectorAll("button").forEach((b) => b.classList.toggle("on", b.dataset.k === k));
    scrub.max = manifest[k].frames.length - 1;
    $("#feed-count").textContent = `${manifest[k].label} · ${manifest[k].frames.length} of ${fmt(manifest[k].count)} frames`;
    show(0);
  }
  tabs.querySelectorAll("button").forEach((b) => (b.onclick = () => selectStream(b.dataset.k)));
  scrub.oninput = () => show(+scrub.value);
  $("#f-prev").onclick = () => show(i - 1);
  $("#f-next").onclick = () => show(i + 1);
  $("#f-play").onclick = () => {
    playing = !playing;
    $("#f-play").textContent = playing ? "❚❚" : "▶";
    if (playing) timer = setInterval(() => show(i + 1), 160);
    else clearInterval(timer);
  };
  selectStream(cur);
}

// ---- 3. THE HERO: Three.js combined reconstruction -------------------------
async function init3D(scene) {
  const host = $("#viewer");
  const hint = host.querySelector(".hint");
  const b = scene.bounds;
  const fr = scene.frame || b;   // robust percentile box for framing
  if (!b) { hint.textContent = "no point cloud exported"; return; }

  const W = host.clientWidth, H = host.clientHeight;
  const three = new THREE.Scene();
  three.background = new THREE.Color(0x060a10);

  const camera = new THREE.PerspectiveCamera(50, W / H, 0.1, 100000);
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(W, H);
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  host.appendChild(renderer.domElement);

  // World uses Z-up (metres). Tell Three.js so orbit + framing feel natural.
  const root = new THREE.Group();
  root.rotation.x = -Math.PI / 2;  // map Z-up world into Three's Y-up view
  three.add(root);

  function worldToView(p) {
    // apply root.rotation.x = -90deg: (x,y,z) -> (x, z, -y)
    return new THREE.Vector3(p[0], p[2], -p[1]);
  }

  // frame on the canal body (robust box), not the far LiDAR tail
  const spanC = Math.max(fr.span[0], fr.span[1]);   // along-canal length
  const radius = spanC * 0.62;

  // canal axis in world XY (downstream +). Fall back to the box's long axis.
  let ax = scene.canal_axis || [1, 0];
  const axLen = Math.hypot(ax[0], ax[1]) || 1;
  ax = [ax[0] / axLen, ax[1] / axLen];

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  const targetV = worldToView(fr.center);
  controls.target.copy(targetV);

  // Oblique "drone shot" from downstream and above, looking back up the canal.
  // Offset downstream along +canal axis, out to one side, and lifted up.
  const cen = fr.center;
  const camWorld = [
    cen[0] + ax[0] * spanC * 0.70 - ax[1] * spanC * 0.28,
    cen[1] + ax[1] * spanC * 0.70 + ax[0] * spanC * 0.28,
    cen[2] + spanC * 0.52,
  ];
  camera.position.copy(worldToView(camWorld));
  controls.update();
  controls.minDistance = radius * 0.2;
  controls.maxDistance = radius * 6;

  // --- point clouds --------------------------------------------------------
  async function loadCloud(meta, mode) {
    if (!meta) return;
    const buf = await (await fetch(`/web/data/${meta.file}`)).arrayBuffer();
    const pos = new Float32Array(buf);
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.BufferAttribute(pos, 3));
    const col = new Float32Array(pos.length);
    // use this cloud's own bounds so the ramp spans its full range
    const cb = meta.bounds;
    if (mode === "lidar") {
      // vivid turbo height ramp: blue near the water → yellow/red up the towers
      const z0 = cb.min[2], zr = (cb.max[2] - cb.min[2]) || 1;
      for (let i = 0; i < pos.length; i += 3) {
        const t = (pos[i + 2] - z0) / zr;
        const c = turbo(t);
        col[i] = c[0]; col[i + 1] = c[1]; col[i + 2] = c[2];
      }
    } else {
      // sonar depth ramp: bright amber just below water → deep red in the scour pit
      const zTop = cb.max[2], zBot = cb.min[2];
      const zr = (zTop - zBot) || 1;
      for (let i = 0; i < pos.length; i += 3) {
        // d = 0 at the surface, 1 at the deepest point
        const d = Math.min(1, Math.max(0, (zTop - pos[i + 2]) / zr));
        // amber (1.0,0.72,0.18) -> red (0.85,0.08,0.05)
        col[i] = 1.0 - 0.15 * d;
        col[i + 1] = 0.72 - 0.64 * d;
        col[i + 2] = 0.18 - 0.13 * d;
      }
    }
    geo.setAttribute("color", new THREE.BufferAttribute(col, 3));
    const size = mode === "lidar" ? radius / 150 : radius / 110;
    const mat = new THREE.PointsMaterial({
      size, vertexColors: true, sizeAttenuation: true,
    });
    const pts = new THREE.Points(geo, mat);
    pts.renderOrder = mode === "sonar" ? 1 : 0;
    root.add(pts);
  }
  await loadCloud(scene.clouds.lidar, "lidar");
  await loadCloud(scene.clouds.sonar, "sonar");

  // --- translucent water plane at z = 0 ------------------------------------
  const waterSize = Math.max(fr.span[0], fr.span[1]) * 1.15;
  const water = new THREE.Mesh(
    new THREE.PlaneGeometry(waterSize, waterSize),
    new THREE.MeshBasicMaterial({ color: 0x1a6fa3, transparent: true, opacity: 0.12, side: THREE.DoubleSide, depthWrite: false })
  );
  water.position.set(fr.center[0], fr.center[1], scene.water_z || 0);
  water.renderOrder = 2;
  root.add(water);
  const waterGrid = new THREE.GridHelper(waterSize, 36, 0x2e7fb0, 0x16384a);
  waterGrid.rotation.x = Math.PI / 2;
  waterGrid.position.set(fr.center[0], fr.center[1], (scene.water_z || 0) + 0.02);
  waterGrid.material.opacity = 0.14; waterGrid.material.transparent = true;
  root.add(waterGrid);

  // --- paths (thick Line2, drawn on top with depthTest off) ----------------
  const lineMaterials = [];
  function addPath(pts, color, widthPx, opacity) {
    if (!pts || pts.length < 2) return;
    const flat = [];
    for (const p of pts) { flat.push(p[0], p[1], p[2]); }
    const g = new LineGeometry();
    g.setPositions(flat);
    const mat = new LineMaterial({
      color, linewidth: widthPx, transparent: true, opacity: opacity ?? 1,
      depthTest: false, dashed: false,
    });
    mat.resolution.set(host.clientWidth, host.clientHeight);
    lineMaterials.push(mat);
    const line = new Line2(g, mat);
    line.computeLineDistances();
    line.renderOrder = 10;
    root.add(line);
  }
  addPath(scene.rov_truth_path, 0xffffff, 3.0, 0.65);  // ROV ground truth, white
  addPath(scene.drone_path, 0x4cc2ff, 4.5, 0.98);      // drone cyan
  addPath(scene.rov_path, 0xff9d42, 4.5, 0.98);        // ROV DVL estimate, amber

  // --- defect markers (GT) + hover labels ----------------------------------
  const markers = [];
  const mkR = radius / 38;
  for (const d of scene.defects) {
    const color = d.underwater ? 0xd65cff : SEV_COLOR[sevName(d.severity)] || 0xff5a5a;
    const m = new THREE.Mesh(
      new THREE.SphereGeometry(mkR, 20, 20),
      new THREE.MeshBasicMaterial({ color }));
    m.position.set(d.pos[0], d.pos[1], d.pos[2]);
    m.renderOrder = 12;
    m.userData = { label: `${d.id} · ${d.type} · sev ${d.severity}${d.underwater ? " (underwater)" : ""} · ${d.element}` };
    root.add(m);
    markers.push(m);
    // soft glow: two translucent additive shells
    for (const [s, op] of [[1.9, 0.30], [3.0, 0.14]]) {
      const glow = new THREE.Mesh(
        new THREE.SphereGeometry(mkR * s, 18, 18),
        new THREE.MeshBasicMaterial({
          color, transparent: true, opacity: op,
          blending: THREE.AdditiveBlending, depthWrite: false,
        }));
      glow.position.copy(m.position);
      glow.renderOrder = 11;
      root.add(glow);
    }
  }

  // --- drone-flag -> ROV dive lines ----------------------------------------
  for (const dl of scene.dive_lines) {
    const col = dl.done ? 0x4ade80 : 0xffd24a;
    const g = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(...dl.top), new THREE.Vector3(...dl.bottom)]);
    const line = new THREE.LineDashedMaterial({ color: col, dashSize: mkR, gapSize: mkR * 0.7, transparent: true, opacity: 0.9 });
    const l = new THREE.Line(g, line); l.computeLineDistances();
    root.add(l);
    // little dive target at the bottom
    const t = new THREE.Mesh(new THREE.SphereGeometry(mkR * 0.7, 12, 12),
      new THREE.MeshBasicMaterial({ color: col }));
    t.position.set(...dl.bottom);
    root.add(t);
  }

  hint.remove();

  // --- hover picking -------------------------------------------------------
  const ray = new THREE.Raycaster(); ray.params.Points = { threshold: mkR };
  const mouse = new THREE.Vector2();
  const hoverEl = $("#hover-label");
  renderer.domElement.addEventListener("pointermove", (ev) => {
    const r = renderer.domElement.getBoundingClientRect();
    mouse.x = ((ev.clientX - r.left) / r.width) * 2 - 1;
    mouse.y = -((ev.clientY - r.top) / r.height) * 2 + 1;
    ray.setFromCamera(mouse, camera);
    const hit = ray.intersectObjects(markers, false)[0];
    if (hit) {
      hoverEl.style.display = "block";
      hoverEl.style.left = `${ev.clientX - r.left + 12}px`;
      hoverEl.style.top = `${ev.clientY - r.top + 10}px`;
      hoverEl.textContent = hit.object.userData.label;
    } else { hoverEl.style.display = "none"; }
  });

  // --- auto-orbit until the user interacts ---------------------------------
  let userInteracted = false;
  controls.addEventListener("start", () => { userInteracted = true; });

  addEventListener("resize", () => {
    const w = host.clientWidth, h = host.clientHeight;
    camera.aspect = w / h; camera.updateProjectionMatrix(); renderer.setSize(w, h);
    for (const m of lineMaterials) m.resolution.set(w, h);
  });

  (function loop() {
    requestAnimationFrame(loop);
    if (!userInteracted) {
      const a = 0.0012;
      const dx = camera.position.x - controls.target.x;
      const dz = camera.position.z - controls.target.z;
      camera.position.x = controls.target.x + dx * Math.cos(a) - dz * Math.sin(a);
      camera.position.z = controls.target.z + dx * Math.sin(a) + dz * Math.cos(a);
    }
    controls.update();
    renderer.render(three, camera);
  })();
}

function sevName(s) {
  if (typeof s === "string") return s;
  return s >= 3 ? "severe" : s === 2 ? "poor" : "fair";
}

// Turbo colormap (Google AI, polynomial approx) — perceptually vivid blue→red.
function turbo(t) {
  t = Math.min(1, Math.max(0, t));
  const r = 0.13572138 + t * (4.61539260 + t * (-42.66032258 + t * (132.13108234 + t * (-152.94239396 + t * 59.28637943))));
  const g = 0.09140261 + t * (2.19418839 + t * (4.84296658 + t * (-14.18503333 + t * (4.27729857 + t * 2.82956604))));
  const b = 0.10667330 + t * (12.64194608 + t * (-60.58204836 + t * (110.36276771 + t * (-89.90310912 + t * 27.34824973))));
  return [Math.min(1, Math.max(0, r)), Math.min(1, Math.max(0, g)), Math.min(1, Math.max(0, b))];
}

main();
