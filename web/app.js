import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

const SEV_COLOR = { severe: 0xff5c5c, poor: 0xffb24a, fair: 0xffe066, good: 0x58d68d };
const SEV_RANK = { good: 0, fair: 1, poor: 2, severe: 3 };

async function getJSON(url) {
  const r = await fetch(url);
  if (!r.ok) throw new Error(`${url}: ${r.status}`);
  return r.json();
}

async function main() {
  let result, cloudMeta, frames;
  try {
    result = await getJSON("/web/data/result.json");
  } catch (e) {
    document.getElementById("mission").textContent = "no data — run export_web.py first";
    return;
  }
  try { cloudMeta = await getJSON("/web/data/cloud.json"); } catch { cloudMeta = null; }
  try { frames = (await getJSON("/web/data/frames.json")).frames; } catch { frames = []; }

  initTabs();
  renderHeader(result);
  renderMetrics(result.metrics);
  renderElementCards(result.detections, frames);
  renderReport(result.report);
  renderTable(result.detections);
  if (cloudMeta) init3D(cloudMeta, result.detections);
  else document.querySelector("#viewer .hint").textContent = "no point cloud exported";
  initFeed(frames);
  renderHealth(result.health || {});
}

// ---- Tabs -------------------------------------------------------------------
function initTabs() {
  const tabs = { defects: document.getElementById("tab-defects"), health: document.getElementById("tab-health") };
  const views = { defects: document.getElementById("view-defects"), health: document.getElementById("view-health") };
  let resized = false;
  function show(name) {
    for (const k of Object.keys(tabs)) {
      tabs[k].classList.toggle("on", k === name);
      views[k].hidden = k !== name;
    }
    if (name === "defects" && !resized) { resized = true; setTimeout(() => dispatchEvent(new Event("resize")), 50); }
  }
  tabs.defects.onclick = () => show("defects");
  tabs.health.onclick = () => show("health");
}

function renderHeader(result) {
  document.getElementById("mission").textContent = result.mission_id;
  document.getElementById("headline").textContent =
    (result.dashboard && result.dashboard.headline) || "";
}

function renderMetrics(m) {
  const cards = [
    ["defects", m.defect_count, ""], ["severe", m.severe, "severe"],
    ["poor", m.poor, "poor"], ["fair", m.fair, "fair"],
    ["new", m.new, ""], ["worsening", m.worsening, ""],
  ];
  document.getElementById("metrics").innerHTML = cards.map(
    ([k, v, cls]) => `<div class="metric ${cls}"><div class="v">${v ?? 0}</div><div class="k">${k}</div></div>`
  ).join("");
}

// ---- Per-element condition cards --------------------------------------------
function renderElementCards(dets, frames) {
  const byEl = {};
  for (const d of dets) {
    const el = d.location.structure.element_id || "(unassigned)";
    (byEl[el] = byEl[el] || []).push(d);
  }
  // frames captured per element (from the frame manifest)
  const framesByEl = {};
  for (const f of (frames || [])) {
    if (f.element) framesByEl[f.element] = (framesByEl[f.element] || 0) + 1;
  }
  const host = document.getElementById("element-cards");
  host.innerHTML = Object.entries(byEl).map(([el, ds]) => {
    const worst = ds.reduce((w, d) => SEV_RANK[d.severity] > SEV_RANK[w] ? d.severity : w, "good");
    const items = ds.map(d => {
      const g = d.geometry.depth_mm != null ? `, ${d.geometry.depth_mm} mm` : "";
      const cf = d.confirmed === true ? " ✓confirmed" : d.confirmed === false ? " (sonar-only)" : "";
      return `<li>${d.class} — ${d.severity}, ${(d.confidence*100).toFixed(0)}%${g}${cf}</li>`;
    }).join("");
    const nframes = framesByEl[el] || 0;
    return `<div class="ecard ${worst}">
      <div class="el">${el}</div>
      <div class="worst ${worst}">worst: ${worst} · ${ds.length} defect(s)</div>
      <ul>${items}</ul>
      <div class="cap">captured: ${nframes} frame(s) · RGB + LiDAR</div>
    </div>`;
  }).join("") || '<div class="hint">no detections</div>';
}

function renderReport(report) {
  const el = document.getElementById("report");
  if (!report) { el.textContent = "—"; return; }
  const txt = (report.narrative || "").replace(
    /\[(D-[0-9a-f]+)\]/g, '<span class="cite">[$1]</span>');
  const isLLM = report.source === "llm";
  const badge = isLLM
    ? `<span class="badge-ai">Claude · ${report.backend}</span>`
    : `<span class="badge-ai">rule-based${report.fallback_reason ? " — " + report.fallback_reason : ""}</span>`;
  const h = document.getElementById("report-h");
  if (h && !h.querySelector(".badge-ai")) h.insertAdjacentHTML("beforeend", " " + badge);
  el.innerHTML = txt;
}

function renderTable(dets) {
  const tb = document.querySelector("#det-table tbody");
  tb.innerHTML = dets.map(d => {
    const s = d.location.structure;
    const geo = d.geometry.depth_mm != null
      ? `${d.geometry.depth_mm} mm${d.geometry.area_mm2 != null ? " / " + d.geometry.area_mm2 + " mm²" : ""}`
      : "—";
    const cf = d.confirmed === true ? "✓" : d.confirmed === false ? "sonar-only" : "—";
    return `<tr>
      <td>${s.element_id || "—"}</td>
      <td>${d.class}</td>
      <td class="sev ${d.severity}">${d.severity}</td>
      <td class="mono">${(d.confidence * 100).toFixed(0)}%</td>
      <td class="mono">${geo}</td>
      <td>${d.change_status || "—"}</td>
      <td class="mono">${cf}</td>
      <td class="mono">${(d.model_version || "").replace(/@.*/, "")}</td>
      <td class="mono">${(d.evidence_frame_ids || []).join(", ")}</td>
    </tr>`;
  }).join("");
}

// ---- 3D LiDAR projection ----------------------------------------------------
async function init3D(meta, dets) {
  const host = document.getElementById("viewer");
  const hint = host.querySelector(".hint");
  document.getElementById("cloud-info").textContent = `${meta.count.toLocaleString()} pts`;

  const w = host.clientWidth || 600, h = host.clientHeight || 460;
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0b0b10);
  const camera = new THREE.PerspectiveCamera(55, w / h, 0.1, 5000);
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(w, h);
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  host.appendChild(renderer.domElement);

  const c = meta.center;
  const span = Math.max(
    meta.bounds.max[0] - meta.bounds.min[0],
    meta.bounds.max[1] - meta.bounds.min[1],
    meta.bounds.max[2] - meta.bounds.min[2]) || 50;

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(c[0], c[1], c[2]);
  camera.position.set(c[0] + span * 0.8, c[1] + span * 0.8, c[2] + span * 0.8);
  controls.update();

  const buf = await (await fetch("/web/data/cloud.bin")).arrayBuffer();
  const pos = new Float32Array(buf);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.BufferAttribute(pos, 3));
  const col = new Float32Array(pos.length);
  const zmin = meta.bounds.min[2], zrange = (meta.bounds.max[2] - zmin) || 1;
  for (let i = 0; i < pos.length; i += 3) {
    const t = (pos[i + 2] - zmin) / zrange;
    col[i] = 0.22 + 0.25 * t; col[i + 1] = 0.29 + 0.2 * t; col[i + 2] = 0.35 + 0.35 * t;
  }
  geo.setAttribute("color", new THREE.BufferAttribute(col, 3));
  scene.add(new THREE.Points(geo, new THREE.PointsMaterial({ size: span / 400, vertexColors: true })));

  for (const d of dets) {
    const p = defectPos(d);
    if (!p) continue;
    const color = SEV_COLOR[d.severity] || 0xffffff;
    const m = new THREE.Mesh(new THREE.SphereGeometry(span / 90, 16, 16),
      new THREE.MeshBasicMaterial({ color }));
    m.position.set(p[0], p[1], p[2]);
    scene.add(m);
  }

  hint.remove();
  addEventListener("resize", () => {
    const w2 = host.clientWidth, h2 = host.clientHeight;
    if (!w2 || !h2) return;
    camera.aspect = w2 / h2; camera.updateProjectionMatrix(); renderer.setSize(w2, h2);
  });
  (function loop() { requestAnimationFrame(loop); controls.update(); renderer.render(scene, camera); })();
}

function defectPos(d) {
  if (d.world_pos) return d.world_pos;
  return d.pose ? [d.pose.x, d.pose.y, d.pose.z] : null;
}

// ---- Inspection feed --------------------------------------------------------
function initFeed(frames) {
  const img = document.getElementById("feed-img");
  const video = document.getElementById("feed-video");
  const overlay = document.getElementById("feed-overlay");
  const label = document.getElementById("feed-label");
  const strip = document.getElementById("filmstrip");
  const stillsCtl = document.getElementById("stills-controls");
  document.getElementById("feed-info").textContent = `${frames.length} frames`;

  const bStills = document.getElementById("mode-stills");
  const bVideo = document.getElementById("mode-video");
  function setMode(m) {
    const v = m === "video";
    bVideo.classList.toggle("on", v); bStills.classList.toggle("on", !v);
    video.style.display = v ? "block" : "none";
    img.style.display = v ? "none" : "block";
    overlay.style.display = v ? "none" : "flex";
    stillsCtl.style.display = v ? "none" : "flex";
    strip.style.display = v ? "none" : "flex";
    if (v && !video.src) video.src = "/web/data/feed.mp4";
  }
  bStills.onclick = () => setMode("stills");
  bVideo.onclick = () => setMode("video");
  fetch("/web/data/feed.mp4", { method: "HEAD" }).then(r => {
    if (!r.ok) { bVideo.disabled = true; bVideo.title = "run annotate_frames.py"; }
  }).catch(() => { bVideo.disabled = true; });

  if (!frames.length) { label.textContent = "no frames"; return; }
  let i = 0, playing = false, timer = null;
  strip.innerHTML = frames.map((f, k) =>
    `<img src="/frames/${f.name}" data-i="${k}" class="${f.detections.length ? "has-defect" : ""}" loading="lazy">`
  ).join("");
  strip.querySelectorAll("img").forEach(t => t.addEventListener("click", () => show(+t.dataset.i)));

  function show(k) {
    i = (k + frames.length) % frames.length;
    const f = frames[i];
    img.src = `/frames/${f.name}`;
    label.textContent = `${f.name} · ${f.element || "—"} · ${f.detections.length} defect(s)`;
    overlay.innerHTML = f.detections.map(d => {
      const c = { severe: "#ff5c5c", poor: "#ffb24a", fair: "#ffe066" }[d.severity] || "#fff";
      return `<span class="tag" style="color:${c};border-color:${c}">${d.class} ${(d.confidence*100).toFixed(0)}%</span>`;
    }).join("");
    strip.querySelectorAll("img").forEach(t => t.classList.toggle("active", +t.dataset.i === i));
    const a = strip.querySelector("img.active");
    if (a) a.scrollIntoView({ inline: "center", block: "nearest" });
  }
  document.getElementById("prev").onclick = () => show(i - 1);
  document.getElementById("next").onclick = () => show(i + 1);
  document.getElementById("play").onclick = (e) => {
    playing = !playing; e.target.textContent = playing ? "❚❚ pause" : "▶ play";
    if (playing) timer = setInterval(() => show(i + 1), 700); else clearInterval(timer);
  };
  show(0);
}

// ---- Tab 2 — Platform health ------------------------------------------------
function renderHealth(health) {
  renderDrone(health.drone);
  renderRov(health.rov);
}

function kpi(k, v, cls = "") {
  return `<div class="metric ${cls}"><div class="v">${v}</div><div class="k">${k}</div></div>`;
}
function bar(label, pct, val) {
  const p = Math.max(0, Math.min(100, pct || 0));
  return `<div class="ubar"><span>${label}</span><span class="track"><span class="fill" style="width:${p}%"></span></span><span class="val">${val}</span></div>`;
}
function row(k, v, cls = "") {
  return `<div class="row"><span class="k">${k}</span><span class="v ${cls}">${v}</span></div>`;
}

function renderDrone(d) {
  document.getElementById("drone-model").textContent = d ? d.model : "no telemetry";
  document.getElementById("drone-badge").innerHTML = '<span class="badge real">real telemetry</span>';
  if (!d) { document.getElementById("drone-kpis").innerHTML = '<div class="hint">telemetry.csv not found</div>'; return; }
  const u = d.utilization, g = d.gnss, l = d.lidar, dq = d.data_quality, mi = d.mission;
  document.getElementById("drone-kpis").innerHTML = [
    kpi("mission", `${mi.duration_s}s`), kpi("inspecting", `${u.inspecting_pct}%`),
    kpi("RTK fixed", `${g.rtk_fixed_pct}%`), kpi("avg sats", g.avg_sats),
    kpi("avg HDOP", g.avg_hdop), kpi("data", dq.grade,
      dq.grade === "high" ? "good" : dq.grade === "degraded" ? "severe" : "fair"),
  ].join("");
  document.getElementById("drone-util").innerHTML =
    bar("Inspecting", u.inspecting_pct, `${u.inspecting_pct}%`) +
    bar("Transit", u.transit_pct, `${u.transit_pct}%`) +
    u.per_element.map(e => bar(e.element, e.share_pct, `${e.seconds}s`)).join("");
  const gq = g.quality === "excellent" || g.quality === "good" ? "good" : g.quality === "fair" ? "warn" : "bad";
  document.getElementById("drone-gnss").innerHTML =
    row("Fix quality", g.quality, gq) + row("RTK-fixed share", `${g.rtk_fixed_pct}%`) +
    row("Avg satellites", g.avg_sats) + row("Avg HDOP", g.avg_hdop,
      g.avg_hdop <= 1.5 ? "good" : g.avg_hdop <= 2.5 ? "warn" : "bad");
  document.getElementById("drone-dq").innerHTML =
    row("LiDAR avg pts/scan", l.avg_points_per_scan.toLocaleString()) +
    row("LiDAR median pts/scan", l.median_points_per_scan.toLocaleString()) +
    row("Low-return steps", `${l.low_return_steps} (${l.low_return_pct}%)`,
      l.low_return_pct > 10 ? "warn" : "good") +
    row("Telemetry rate", `${mi.telemetry_hz} Hz`) +
    row("Overall data quality", dq.grade, dq.grade === "high" ? "good" : dq.grade === "degraded" ? "bad" : "warn") +
    (dq.issues.length ? `<ul class="issues">${dq.issues.map(x => `<li>${x}</li>`).join("")}</ul>` : "");
}

function renderRov(r) {
  const panel = document.getElementById("rov-panel");
  document.getElementById("rov-model").textContent = r ? r.model : "n/a";
  if (r && r.is_filler) {
    panel.classList.add("filler");
    document.getElementById("rov-badge").innerHTML = '<span class="badge filler">filler — not flown</span>';
  }
  if (!r) return;
  const u = r.utilization;
  document.getElementById("rov-kpis").innerHTML = [
    kpi("mission", `${r.mission.duration_s}s`), kpi("inspecting", `${u.inspecting_pct}%`),
    kpi("dives", "0"), kpi("confirms", "—"), kpi("coverage", "—"), kpi("data", "n/a"),
  ].join("");
  document.getElementById("rov-util").innerHTML =
    (u.per_element || []).map(e => bar(e.element, e.share_pct, `${e.seconds}s`)).join("") ||
    '<div class="hint">no ROV deployment in this mission</div>';
  document.getElementById("rov-dq").innerHTML =
    row("Sonar", (r.sonar && r.sonar.note) || "—") +
    row("Camera", (r.camera && r.camera.note) || "—") +
    (r.data_quality.issues.length ? `<ul class="issues">${r.data_quality.issues.map(x => `<li>${x}</li>`).join("")}</ul>` : "");
}

main();
