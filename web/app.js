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
  renderElementCards(result.detections, frames, result.health || {});
  renderReport(result.report);
  renderTable(result.detections);
  if (cloudMeta) init3D(cloudMeta, result.detections);
  else document.querySelector("#viewer .hint").textContent = "no point cloud exported";
  initFeed(frames);
  renderHealth(result.health || {});
  initDetail();
  initReview();
  try { renderEval(await getJSON("/web/data/eval.json")); } catch { /* no evaluation exported */ }
}

// ---- Detection detail drawer --------------------------------------------------
// Static reference text: what each class is. Presentation only; no values here.
const CLASS_INFO = {
  crack: "Linear fracture in the concrete or masonry surface. Width and length drive concern; "
    + "water entering cracks accelerates corrosion and freeze-thaw damage.",
  spall: "Concrete or masonry broken away from the surface, leaving a cavity. Often caused by "
    + "corroding reinforcement expanding beneath the cover.",
  exposed_rebar: "Reinforcing steel visible at the surface: the protective cover is already gone, "
    + "so corrosion is unchecked. Serious on presence alone.",
  corrosion_stain: "Rust-coloured staining from corroding steel. An early indicator that embedded "
    + "steel is corroding; not section loss on its own.",
  efflorescence: "White salt deposits left by water migrating through the concrete. Indicates a "
    + "water path; cosmetic by itself, but water drives most deterioration.",
  rust_streak: "Rust streaks running down the surface from a steel source (bearings, girders, "
    + "embedded steel). Early corrosion indicator.",
  scour: "Removal of bed material around a foundation by flowing water. Scour is the leading cause "
    + "of bridge collapse in the US; undermines bearing support.",
  section_loss: "Measured loss of member cross-section (steel or concrete), reducing capacity.",
  pier_damage: "Impact or deterioration damage on a pier below the water line.",
  missing_block: "A masonry block dislodged from the wall below the water line. Already a loss of "
    + "section; water can now scour behind the face.",
  undermining: "Material washed out from beneath a footing, leaving it partly unsupported.",
  sonar_anomaly: "An unusual sonar return flagged by a generic seabed-hazard sonar model (not "
    + "trained on bridge defects). It says 'something is here', not what it is; an inspector "
    + "must identify it.",
};

let DETS_BY_ID = {};

function initDetail() {
  const close = () => {
    document.getElementById("detail").hidden = true;
    document.getElementById("detail-backdrop").hidden = true;
  };
  document.getElementById("detail-close").onclick = close;
  document.getElementById("detail-backdrop").onclick = close;
  addEventListener("keydown", e => { if (e.key === "Escape") close(); });
  document.querySelectorAll("#det-table tbody tr").forEach(tr =>
    tr.addEventListener("click", () => openDetail(DETS_BY_ID[tr.dataset.id])));
}

function esc(s) {
  return String(s ?? "").replace(/[&<>"]/g, c => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
}

function fmtNum(v, digits = 2) {
  return v === null || v === undefined ? "—" : Number(v).toFixed(digits);
}

function measurementText(d) {
  const g = d.geometry || {};
  if (g.depth_mm == null && g.area_mm2 == null) {
    return d.platform === "rov"
      ? "No measurement: too few sonar-map points around this position to fit a surface."
      : "No measurement: too few LiDAR-map points around this position to fit a surface.";
  }
  const src = d.platform === "rov" ? "the ROV's accumulated sonar map" : "the drone's accumulated LiDAR map";
  let t = `Local surface roughness ${fmtNum(g.depth_mm, 1)} mm: the RMS distance of points in a 0.6 m box `
    + `around the defect from a plane fitted through them, measured on ${src}. `
    + "It shows how uneven the surface is there; it is not a cavity depth, and no area is claimed.";
  if (g.area_mm2 != null) t += ` Area ${fmtNum(g.area_mm2, 0)} mm².`;
  return t;
}

function captureText(d) {
  const mv = d.model_version || "";
  const ev = d.evidence_frame_ids || [];
  const parts = [];
  if (mv.startsWith("ground-truth")) {
    parts.push("Taken from the mission's ground-truth file (defects_gt.json), not from a detector. "
      + "Used to check the pipeline; it says nothing about detector accuracy.");
  } else if (d.platform === "drone") {
    parts.push(`Detected by the drone camera model (${esc(mv)}) on frame ${esc(d.source_frame_id)}.`);
  } else {
    parts.push(`Detected by the ROV sonar model (${esc(mv)}) on ping ${esc(d.source_frame_id)}`
      + (ev.length > 1 ? `, seen on ${ev.length} pings` : "") + ".");
  }
  if (d.platform === "rov") {
    if (d.confirmed === true) parts.push("Confirmed by a co-located ROV camera frame that cleared the visibility gate.");
    else if (d.confirmed === false) parts.push("Sonar-only: no co-located ROV camera frame was clear enough to confirm it (turbid water).");
  } else {
    parts.push("Located by projecting from the drone along the camera's aim line by the measured standoff.");
  }
  if (ev.length) parts.push(`Evidence: ${ev.slice(0, 10).map(esc).join(", ")}${ev.length > 10 ? " …" : ""}.`);
  return parts.join(" ");
}

function reliabilityText(d) {
  const c = d.confidence;
  if ((d.model_version || "").startsWith("ground-truth")) return "Ground truth (reference data).";
  if (c < 0.4) return `Tentative: model confidence ${(c * 100).toFixed(0)}% is below 40%. Field verification needed before acting.`;
  if (c < 0.6) return `Moderate: model confidence ${(c * 100).toFixed(0)}%.`;
  return `Model confidence ${(c * 100).toFixed(0)}%.`;
}

function openDetail(d) {
  if (!d) return;
  const s = d.location.structure, wp = d.world_pos;
  const depth = s.elevation_or_depth;
  const rows = [
    ["Element", esc(s.element_id || "unassigned")],
    ["Platform", d.platform === "rov" ? "ROV (underwater)" : "Drone (above water)"],
    ["World position (m)", wp ? wp.map(v => fmtNum(v, 2)).join(", ") : "—"],
    ...(d.platform === "rov" && depth != null ? [["Elevation (m, water line = 0)", fmtNum(depth, 2)]] : []),
    ["Change vs last cycle", esc(d.change_status || "—")],
    ["Detection ID", `<code>${esc(d.detection_id)}</code>`],
  ];
  document.getElementById("detail-body").innerHTML = `
    <div class="detail-head">
      <span class="sev-pill ${esc(d.severity)}">${esc(d.severity)}</span>
      <h3>${esc(d.class)}</h3>
    </div>
    <p class="detail-what">${CLASS_INFO[d.class] || "No description for this class."}</p>

    <h4>Why this severity</h4>
    <p>${d.severity_reason ? esc(d.severity_reason).replace(/; /g, "<br>→ ")
        : "Not recorded in this result file (re-run the pipeline to include the rule trail)."}</p>

    <h4>Recommended action</h4>
    <p class="detail-action">${esc(d.recommended_action || "Not recorded in this result file.")}</p>

    <h4>Measurement</h4>
    <p>${measurementText(d)}</p>

    <h4>How it was captured</h4>
    <p>${captureText(d)}</p>

    <h4>Reliability</h4>
    <p>${reliabilityText(d)}</p>

    <h4>Location &amp; record</h4>
    <div class="kv">${rows.map(([k, v]) => `<div class="row"><span class="k">${k}</span><span class="v">${v}</span></div>`).join("")}</div>
  `;
  document.getElementById("detail").hidden = false;
  document.getElementById("detail-backdrop").hidden = false;
}

// ---- Sign-off gate (server enforces the rules: pipeline/review.py) -----------
const REVIEW_LABEL = { draft: "Draft", under_review: "Under review", approved: "Approved", rejected: "Rejected" };
const REVIEW_ACTIONS = {
  draft: [["submit", "Submit for review", false]],
  under_review: [["approve", "Approve", true], ["reject", "Reject", true]],
  rejected: [["resubmit", "Resubmit", false]],
  approved: [],
};

async function initReview() {
  const host = document.getElementById("review");
  let rec;
  try { rec = await getJSON("/api/review"); } catch { host.innerHTML = ""; return; }
  const draw = (r, err = "") => {
    const last = (r.history || []).slice(-1)[0];
    const acts = REVIEW_ACTIONS[r.state] || [];
    const needsName = acts.some(a => a[2]);
    host.innerHTML = `
      <div class="review-state ${r.state}">${REVIEW_LABEL[r.state]}</div>
      <div class="review-meta">${last ? `${esc(last.action)} by ${esc(last.reviewer || "—")} · ${esc(last.at)}`
        + (last.note ? ` · “${esc(last.note)}”` : "") : "not yet submitted for sign-off"}
        ${r.note ? `<br><span class="warn">${esc(r.note)}</span>` : ""}
        <br><span class="mono">report ${esc(r.report_hash)}</span></div>
      ${acts.length ? `<div class="review-form">
        ${needsName ? '<input id="rv-name" placeholder="Reviewer (licensed inspector / PE)">' : ""}
        <input id="rv-note" placeholder="Note (optional)">
        ${acts.map(([a, label]) => `<button data-a="${a}" class="rv-${a}">${label}</button>`).join("")}
      </div>` : '<div class="review-meta">Signed off. Re-running the pipeline with different results resets this to draft.</div>'}
      ${err ? `<div class="review-err">${esc(err)}</div>` : ""}`;
    host.querySelectorAll("button[data-a]").forEach(b => b.onclick = async () => {
      const body = { action: b.dataset.a, report_hash: r.report_hash,
        reviewer: (host.querySelector("#rv-name") || {}).value || "",
        note: (host.querySelector("#rv-note") || {}).value || "" };
      const resp = await fetch("/api/review", { method: "POST", headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body) });
      const data = await resp.json();
      if (resp.ok) draw(data); else draw(r, data.error || "review failed");
    });
  };
  draw(rec);
}

// ---- Evaluation panel (optional: web/data/eval.json) -------------------------
function renderEval(e) {
  if (!e) return;
  const panel = document.getElementById("eval-panel");
  const line = (name, m, gt) => `<tr><td>${esc(name)}</td><td class="mono">${gt ?? "—"}</td>
    <td class="mono">${m.tp}</td><td class="mono">${m.fp}</td><td class="mono">${m.fn}</td>
    <td class="mono">${fmtNum(m.precision)}</td><td class="mono">${fmtNum(m.recall)}</td></tr>`;
  const rows = [line("all (class-agnostic)", e.class_agnostic, e.ground_truth && e.ground_truth.count),
                line("all (class must match)", e.class_aware, e.ground_truth && e.ground_truth.count)];
  for (const [p, v] of Object.entries(e.per_platform || {})) rows.push(line(p, v.class_agnostic, v.gt));
  const note = e.gt_sourced_predictions
    ? `<p class="help warn">${e.gt_sourced_predictions} of the detections are ground-truth-sourced, so these scores are a pipeline self-check, not detector accuracy.</p>` : "";
  document.getElementById("eval-body").innerHTML = `${note}
    <table><thead><tr><th>Scope</th><th>GT</th><th>TP</th><th>FP</th><th>FN</th><th>Precision</th><th>Recall</th></tr></thead>
    <tbody>${rows.join("")}</tbody></table>
    <p class="help">Match radius ${fmtNum(e.radius_m, 1)} m. Missed ground-truth defects:
      ${(e.class_agnostic.missed || []).map(esc).join(", ") || "none"}.</p>`;
  panel.hidden = false;
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
    ["Defects found", m.defect_count, "", `${m.drone ?? 0} by drone, ${m.rov ?? 0} by ROV`],
    ["Severe", m.severe, "severe", "repair priority now"],
    ["Poor", m.poor, "poor", "verify on next visit"],
    ["Fair", m.fair, "fair", "monitor next cycle"],
    ["New this cycle", m.new, "", "not in the prior inspection"],
    ["Worsening", m.worsening, "", "grown since last cycle"],
  ];
  document.getElementById("metrics").innerHTML = cards.map(([k, v, cls, sub]) =>
    `<div class="metric ${cls}"><div class="v">${v ?? 0}</div><div class="k">${k}</div><div class="s">${sub}</div></div>`
  ).join("");
}

// ---- Per-element condition cards --------------------------------------------
function renderElementCards(dets, frames, health) {
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
  // seconds spent on each element (drone telemetry; ROV status areas)
  const secsByEl = {};
  for (const p of ["drone", "rov"]) {
    const h = health[p];
    if (h && !h.is_filler) for (const e of (h.utilization.per_element || [])) secsByEl[e.element] = e.seconds;
  }
  const host = document.getElementById("element-cards");
  host.innerHTML = Object.entries(byEl).map(([el, ds]) => {
    const worstDet = ds.reduce((w, d) => (SEV_RANK[d.severity] ?? -1) > (SEV_RANK[w.severity] ?? -1) ? d : w, ds[0]);
    const worst = worstDet.severity || "good";
    const counts = {};
    for (const d of ds) counts[d.class] = (counts[d.class] || 0) + 1;
    const summary = Object.entries(counts).map(([c, n]) => `${n} ${c}`).join(", ");
    const tentative = ds.filter(d => d.confidence < 0.4 && !(d.model_version || "").startsWith("ground-truth")).length;
    const items = ds.map(d => {
      const g = d.geometry.depth_mm != null ? `, ${d.geometry.depth_mm} mm roughness` : "";
      const cf = d.confirmed === true ? " ✓camera-confirmed" : d.confirmed === false ? " (sonar-only)" : "";
      return `<li class="clickable" data-id="${esc(d.detection_id)}">${esc(d.class)} — ${esc(d.severity)}, `
        + `${(d.confidence*100).toFixed(0)}%${g}${cf}</li>`;
    }).join("");
    const nframes = framesByEl[el] || 0;
    const plats = [...new Set(ds.map(d => d.platform))];
    const sensors = plats.map(p => p === "rov" ? "sonar + ROV camera" : "RGB + LiDAR").join(" / ");
    const secs = secsByEl[el] != null ? ` · ${secsByEl[el]} s on element` : "";
    return `<div class="ecard ${worst}">
      <div class="el">${esc(el)}</div>
      <div class="worst ${worst}">worst: ${worst} · ${ds.length} defect(s)</div>
      <p class="esum">${esc(summary)}.${tentative ? ` ${tentative} tentative (confidence &lt; 40%).` : ""}</p>
      <ul>${items}</ul>
      <p class="eaction"><b>Action:</b> ${esc(worstDet.recommended_action || "re-run the pipeline to include recommendations")}</p>
      <div class="cap">captured: ${nframes} frame(s) · ${sensors}${secs}</div>
    </div>`;
  }).join("") || '<div class="hint">no detections</div>';
  host.querySelectorAll("li.clickable").forEach(li =>
    li.addEventListener("click", () => openDetail(dets.find(d => d.detection_id === li.dataset.id))));
}

function renderReport(report) {
  const el = document.getElementById("report");
  if (!report) { el.textContent = "—"; return; }
  const txt = (report.narrative || "").replace(
    /\[(D-[0-9a-f]+)\]/g, '<span class="cite">[$1]</span>');
  const isLLM = report.source === "llm";
  const BACKEND_LABEL = { claude: "Claude API", kiro: "Kiro CLI (Kiro credits)", local: "local model" };
  const badge = isLLM
    ? `<span class="badge-ai">AI draft · ${BACKEND_LABEL[report.backend] || report.backend}</span>`
    : `<span class="badge-ai">rule-based${report.fallback_reason ? " — " + report.fallback_reason : ""}</span>`;
  const h = document.getElementById("report-h");
  if (h && !h.querySelector(".badge-ai")) h.insertAdjacentHTML("beforeend", " " + badge);
  el.innerHTML = txt;
}

function renderTable(dets) {
  const tb = document.querySelector("#det-table tbody");
  DETS_BY_ID = Object.fromEntries(dets.map(d => [d.detection_id, d]));
  tb.innerHTML = dets.map(d => {
    const s = d.location.structure;
    const geo = d.geometry.depth_mm != null
      ? `${d.geometry.depth_mm} mm${d.geometry.area_mm2 != null ? " / " + d.geometry.area_mm2 + " mm²" : ""}`
      : "—";
    const cf = d.confirmed === true ? "✓" : d.confirmed === false ? "sonar-only" : "—";
    return `<tr data-id="${d.detection_id}" class="clickable" title="click for details">
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

  const markers = [];
  for (const d of dets) {
    const p = defectPos(d);
    if (!p) continue;
    const color = SEV_COLOR[d.severity] || 0xffffff;
    const m = new THREE.Mesh(new THREE.SphereGeometry(span / 90, 16, 16),
      new THREE.MeshBasicMaterial({ color }));
    m.position.set(p[0], p[1], p[2]);
    m.userData.detection = d;
    markers.push(m);
    scene.add(m);
  }

  // click a marker -> detail drawer (ignore drags used to orbit)
  const ray = new THREE.Raycaster(), ndc = new THREE.Vector2();
  let down = null;
  renderer.domElement.addEventListener("pointerdown", e => { down = [e.clientX, e.clientY]; });
  renderer.domElement.addEventListener("pointerup", e => {
    if (!down || Math.hypot(e.clientX - down[0], e.clientY - down[1]) > 4) return;
    const r = renderer.domElement.getBoundingClientRect();
    ndc.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
    ray.setFromCamera(ndc, camera);
    const hit = ray.intersectObjects(markers)[0];
    if (hit) openDetail(hit.object.userData.detection);
  });
  renderer.domElement.style.cursor = "pointer";

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
  renderHardware(health.hardware);
  renderDrone(health.drone);
  renderRov(health.rov);
}

// ---- Hardware health (simulated telemetry) ----------------------------------
const HW_LABEL = { ok: "OK", warn: "WARN", fault: "FAULT" };

function batterySpark(series) {
  if (!series || !series.length) return "";
  const W = 260, H = 54, tmax = series[series.length - 1].t_s || 1;
  const pts = series.map(p => `${(p.t_s / tmax * W).toFixed(1)},${(H - p.pct / 100 * H).toFixed(1)}`).join(" ");
  const warnY = H - 30 / 100 * H;
  return `<svg class="spark" viewBox="0 0 ${W} ${H}" preserveAspectRatio="none" aria-label="battery over mission">
    <line x1="0" x2="${W}" y1="${warnY}" y2="${warnY}" class="spark-warn"/>
    <polyline points="${pts}" class="spark-line"/></svg>`;
}

function renderHwPlatform(host, h, title) {
  if (!h) {
    host.innerHTML = `<h2>${title}</h2><p class="help">Not deployed on this mission — nothing to monitor.</p>`;
    return;
  }
  const b = h.battery;
  const comps = h.components.map(c => `
    <div class="hw-row ${c.status}">
      <span class="hw-dot"></span><span class="hw-name">${esc(c.name)}</span>
      <span class="hw-val">${esc(c.value)}</span><span class="hw-st">${HW_LABEL[c.status]}</span>
      <span class="hw-note">${esc(c.note)}</span>
    </div>`).join("");
  const motors = h.motors.map(m => `
    <div class="motor ${m.status}" title="${esc(m.name)}">
      <div class="mt">${m.temp_c}°</div><div class="mn">${esc(m.name)}</div>
      <div class="mx">${m.rpm.toLocaleString()} rpm · ${m.current_a} A</div>
    </div>`).join("");
  host.innerHTML = `
    <h2>${title} <small>${esc(h.model)}</small>
      <span class="hw-overall ${h.overall}">${HW_LABEL[h.overall]}</span></h2>
    <div class="hw-batt">
      <div class="hw-batt-num ${b.status}"><div class="v">${b.now_pct}%</div>
        <div class="k">battery now</div><div class="s">${b.now_voltage_v} V · ${b.used_mah} mAh used</div></div>
      <div class="hw-batt-chart">${batterySpark(b.series)}
        <div class="s">charge over the mission · dashed line = ${h.thresholds.battery_warn_pct}% return threshold</div>
        <div class="cells">${b.cell_voltages_v.map((v, i) => `<span>C${i + 1} ${v.toFixed(2)} V</span>`).join("")}
          <span class="${b.cell_spread_v >= 0.05 ? "warn" : ""}">spread ${b.cell_spread_v} V</span></div>
      </div>
    </div>
    <h3>${esc(h.motor_label)} <small>case temperature · warn ≥ ${h.thresholds.motor_temp_warn_c}°C,
      fault ≥ ${h.thresholds.motor_temp_fault_c}°C</small></h3>
    <div class="motors">${motors}</div>
    <h3>Components</h3>
    <div class="hw-list">${comps}</div>`;
}

function renderHardware(hw) {
  const banner = document.getElementById("hw-banner");
  if (!hw) { banner.querySelector("#hw-alerts").innerHTML =
    '<div class="hint">No hardware block in this result — re-run the pipeline.</div>'; return; }
  renderHwPlatform(document.getElementById("hw-drone"), hw.drone, "Drone hardware");
  renderHwPlatform(document.getElementById("hw-rov"), hw.rov, "ROV hardware");
  const alerts = [["Drone", hw.drone], ["ROV", hw.rov]].flatMap(([p, h]) =>
    (h ? h.alerts : []).map(a => ({ ...a, p })));
  document.getElementById("hw-alerts").innerHTML = alerts.length
    ? `<ul class="hw-alerts">${alerts.map(a =>
        `<li class="${a.level}"><b>${HW_LABEL[a.level]} · ${a.p}</b> ${esc(a.text)}</li>`).join("")}</ul>`
    : '<div class="hw-allclear">All monitored components within limits.</div>';
}

// KPI subtitles: one short line saying what each number is.
const KPI_HINT = {
  "mission": "recorded duration", "inspecting": "time aimed at an element",
  "RTK fixed": "cm-level GPS share", "avg sats": "GNSS satellites in view",
  "avg HDOP": "GPS geometry, lower is better", "data": "overall data quality",
  "coop dives": "dives tasked by the drone", "confirms": "sonar hits seen on camera",
  "DVL lock": "underwater nav lock held",
};
function kpi(k, v, cls = "") {
  const s = KPI_HINT[k] ? `<div class="s">${KPI_HINT[k]}</div>` : "";
  return `<div class="metric ${cls}"><div class="v">${v}</div><div class="k">${k}</div>${s}</div>`;
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
  if (r && !r.is_filler) {
    panel.classList.remove("filler");
    document.getElementById("rov-badge").innerHTML = '<span class="badge real">real telemetry</span>';
  }
  if (!r) return;
  const u = r.utilization, nav = r.nav || {}, cam = r.camera || {}, dq = r.data_quality;
  const fmt = (v, unit = "") => (v === null || v === undefined ? "—" : `${v}${unit}`);
  document.getElementById("rov-kpis").innerHTML = [
    kpi("mission", `${r.mission.duration_s}s`), kpi("inspecting", `${u.inspecting_pct}%`),
    kpi("coop dives", fmt(r.dives, "")), kpi("confirms", fmt(cam.confirmed_pct, "%")),
    kpi("DVL lock", fmt(nav.dvl_lock_pct, "%")),
    kpi("data", dq.grade, dq.grade === "high" ? "good" : dq.grade === "degraded" ? "severe" : "fair"),
  ].join("");
  document.getElementById("rov-util").innerHTML =
    (u.per_element || []).map(e => bar(e.element, e.share_pct, `${e.seconds}s`)).join("") ||
    '<div class="hint">no ROV deployment in this mission</div>';
  document.getElementById("rov-dq").innerHTML =
    (r.is_filler ? "" :
      row("Max depth", fmt(nav.max_depth_m, " m")) +
      row("DVL drift vs truth (avg / max)", `${fmt(nav.avg_drift_m, " m")} / ${fmt(nav.max_drift_m, " m")}`,
          nav.max_drift_m > 1.0 ? "warn" : "good") +
      row("Camera visibility (avg RMS contrast)", fmt(cam.avg_visibility)) +
      row("Frames clearing the visibility gate", fmt(cam.clear_pct, "%"),
          cam.clear_pct < 50 ? "warn" : "good")) +
    row("Sonar", (r.sonar && r.sonar.note) || "—") +
    row("Camera", cam.note || "—") +
    (dq.issues.length ? `<ul class="issues">${dq.issues.map(x => `<li>${x}</li>`).join("")}</ul>` : "");
}

main();
