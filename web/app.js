import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

const SEV_COLOR = { severe: 0xff5c5c, poor: 0xffb24a, fair: 0xffe066, good: 0x58d68d };

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

  renderHeader(result);
  renderMetrics(result.metrics);
  renderReport(result.report);
  renderTable(result.detections);
  if (cloudMeta) init3D(cloudMeta, result.detections);
  else document.querySelector("#viewer .hint").textContent = "no point cloud exported";
  initFeed(frames);
}

function renderHeader(result) {
  document.getElementById("mission").textContent = result.mission_id;
  document.getElementById("headline").textContent =
    (result.dashboard && result.dashboard.headline) || "";
}

function renderMetrics(m) {
  const cards = [
    ["defects", m.defect_count, ""],
    ["severe", m.severe, "severe"],
    ["poor", m.poor, "poor"],
    ["fair", m.fair, "fair"],
    ["new", m.new, ""],
    ["worsening", m.worsening, ""],
  ];
  document.getElementById("metrics").innerHTML = cards.map(
    ([k, v, cls]) => `<div class="metric ${cls}"><div class="v">${v ?? 0}</div><div class="k">${k}</div></div>`
  ).join("");
}

function renderReport(report) {
  const el = document.getElementById("report");
  if (!report) { el.textContent = "—"; return; }
  const txt = (report.narrative || "").replace(
    /\[(D-[0-9a-f]+)\]/g, '<span class="cite">[$1]</span>');
  const isLLM = report.source === "llm";
  const badge = isLLM
    ? `<span class="badge-ai">Claude · ${report.backend}</span>`
    : `<span class="badge-ai">rule-based (no ${report.backend} key)</span>`;
  // Put the badge next to the panel heading.
  const h = el.previousElementSibling;
  if (h && h.tagName === "H2" && !h.querySelector(".badge-ai")) h.insertAdjacentHTML("beforeend", " " + badge);
  el.innerHTML = txt;
}

function renderTable(dets) {
  const tb = document.querySelector("#det-table tbody");
  tb.innerHTML = dets.map(d => {
    const s = d.location.structure;
    const g = d.geometry.depth_mm != null ? `${d.geometry.depth_mm} mm` : "—";
    const ev = (d.evidence_frame_ids || []).join(", ");
    return `<tr>
      <td>${s.element_id || "—"}</td>
      <td>${d.class}</td>
      <td class="sev ${d.severity}">${d.severity}</td>
      <td class="mono">${(d.confidence * 100).toFixed(0)}%</td>
      <td class="mono">${g}</td>
      <td>${d.change_status || "—"}</td>
      <td class="mono">${ev}</td>
    </tr>`;
  }).join("");
}

// ---- 3D LiDAR projection ----------------------------------------------------
async function init3D(meta, dets) {
  const host = document.getElementById("viewer");
  const hint = host.querySelector(".hint");
  document.getElementById("cloud-info").textContent = `${meta.count.toLocaleString()} pts`;

  const w = host.clientWidth, h = host.clientHeight;
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0a0e14);
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

  // Load the downsampled cloud (Float32 x,y,z triples).
  const buf = await (await fetch("/web/data/cloud.bin")).arrayBuffer();
  const pos = new Float32Array(buf);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.BufferAttribute(pos, 3));
  // Height-based tint so the structure reads in 3D.
  const col = new Float32Array(pos.length);
  const zmin = meta.bounds.min[2], zrange = (meta.bounds.max[2] - zmin) || 1;
  for (let i = 0; i < pos.length; i += 3) {
    const t = (pos[i + 2] - zmin) / zrange;
    col[i] = 0.22 + 0.25 * t; col[i + 1] = 0.29 + 0.2 * t; col[i + 2] = 0.35 + 0.35 * t;
  }
  geo.setAttribute("color", new THREE.BufferAttribute(col, 3));
  const pts = new THREE.Points(geo, new THREE.PointsMaterial({ size: span / 400, vertexColors: true }));
  scene.add(pts);

  // Defect markers at their world positions, colored by severity.
  for (const d of dets) {
    const p = defectPos(d);
    if (!p) continue;
    const color = SEV_COLOR[d.severity] || 0xffffff;
    const m = new THREE.Mesh(
      new THREE.SphereGeometry(span / 90, 16, 16),
      new THREE.MeshBasicMaterial({ color }));
    m.position.set(p[0], p[1], p[2]);
    scene.add(m);
    const ring = new THREE.Mesh(
      new THREE.RingGeometry(span / 70, span / 55, 24),
      new THREE.MeshBasicMaterial({ color, side: THREE.DoubleSide, transparent: true, opacity: 0.5 }));
    ring.position.copy(m.position);
    ring.lookAt(camera.position);
    scene.add(ring);
  }

  hint.remove();
  addEventListener("resize", () => {
    const w2 = host.clientWidth, h2 = host.clientHeight;
    camera.aspect = w2 / h2; camera.updateProjectionMatrix(); renderer.setSize(w2, h2);
  });
  (function loop() { requestAnimationFrame(loop); controls.update(); renderer.render(scene, camera); })();
}

function defectPos(d) {
  if (d.world_pos) return d.world_pos;           // set by the pipeline
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

  // Mode toggle: annotated stills vs. encoded video.
  const bStills = document.getElementById("mode-stills");
  const bVideo = document.getElementById("mode-video");
  function setMode(m) {
    const videoOn = m === "video";
    bVideo.classList.toggle("on", videoOn);
    bStills.classList.toggle("on", !videoOn);
    video.style.display = videoOn ? "block" : "none";
    img.style.display = videoOn ? "none" : "block";
    overlay.style.display = videoOn ? "none" : "flex";
    stillsCtl.style.display = videoOn ? "none" : "flex";
    strip.style.display = videoOn ? "none" : "flex";
    if (videoOn && !video.src) video.src = "/web/data/feed.mp4";
  }
  bStills.onclick = () => setMode("stills");
  bVideo.onclick = () => setMode("video");
  // Probe whether a video exists; disable the toggle if not.
  fetch("/web/data/feed.mp4", { method: "HEAD" }).then(r => {
    if (!r.ok) { bVideo.disabled = true; bVideo.title = "run annotate_frames.py to generate"; }
  }).catch(() => { bVideo.disabled = true; });

  if (!frames.length) { label.textContent = "no frames"; return; }

  let i = 0, playing = false, timer = null;
  strip.innerHTML = frames.map((f, k) =>
    `<img src="/frames/${f.name}" data-i="${k}" class="${f.detections.length ? "has-defect" : ""}" loading="lazy">`
  ).join("");
  strip.querySelectorAll("img").forEach(t =>
    t.addEventListener("click", () => show(+t.dataset.i)));

  function show(k) {
    i = (k + frames.length) % frames.length;
    const f = frames[i];
    img.src = `/frames/${f.name}`;
    label.textContent = `${f.name} · ${f.element || "—"} · ${f.detections.length} defect(s)`;
    overlay.innerHTML = f.detections.map(d => {
      const c = { severe: "#ff5c5c", poor: "#ffb24a", fair: "#ffe066" }[d.severity] || "#fff";
      return `<span class="tag" style="color:${c};border-color:${c}">${d.class} ${(d.confidence * 100).toFixed(0)}%</span>`;
    }).join("");
    strip.querySelectorAll("img").forEach(t => t.classList.toggle("active", +t.dataset.i === i));
    const active = strip.querySelector("img.active");
    if (active) active.scrollIntoView({ inline: "center", block: "nearest" });
  }
  document.getElementById("prev").onclick = () => show(i - 1);
  document.getElementById("next").onclick = () => show(i + 1);
  document.getElementById("play").onclick = (e) => {
    playing = !playing;
    e.target.textContent = playing ? "❚❚ pause" : "▶ play";
    if (playing) timer = setInterval(() => show(i + 1), 700);
    else clearInterval(timer);
  };
  show(0);
}

main();
