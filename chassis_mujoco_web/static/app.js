import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import URDFLoader from "/vendor/urdf-loader/URDFLoader.js";

const canvas = document.getElementById("view");
const badge = document.getElementById("liveBadge");
const bootErr = document.getElementById("bootErr");

function showErr(msg) {
  if (!bootErr) return;
  bootErr.hidden = false;
  bootErr.textContent = msg;
}
window.addEventListener("error", (e) => showErr(`前端错误: ${e.message || e}`));
window.addEventListener("unhandledrejection", (e) =>
  showErr(`前端错误: ${e.reason || e}`)
);

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setClearColor(0x12151a, 1);
renderer.shadowMap.enabled = true;
renderer.outputColorSpace = THREE.SRGBColorSpace;

const scene = new THREE.Scene();
scene.fog = new THREE.Fog(0x12151a, 12, 30);

const camera = new THREE.PerspectiveCamera(50, 1, 0.05, 80);
camera.position.set(2.6, 1.6, 2.8);

const controls = new OrbitControls(camera, canvas);
controls.target.set(0, 0.2, 0);
controls.enableDamping = true;
controls.maxPolarAngle = Math.PI * 0.49;

scene.add(new THREE.HemisphereLight(0xe8eef5, 0x2a3038, 1.05));
const key = new THREE.DirectionalLight(0xfff2d6, 1.2);
key.position.set(3.5, 6, 2.5);
key.castShadow = true;
scene.add(key);
const fill = new THREE.DirectionalLight(0x9eb6c8, 0.4);
fill.position.set(-3, 2, -2);
scene.add(fill);

const grid = new THREE.GridHelper(16, 32, 0x3a4450, 0x252b33);
grid.position.y = 0.001;
scene.add(grid);
const ground = new THREE.Mesh(
  new THREE.PlaneGeometry(16, 16),
  new THREE.MeshStandardMaterial({ color: 0x1a1f26, roughness: 0.95, metalness: 0.05 })
);
ground.rotation.x = -Math.PI / 2;
ground.receiveShadow = true;
scene.add(ground);

/** URDF is ROS Z-up; rotate once into Three Y-up. */
const rosRoot = new THREE.Group();
rosRoot.rotation.x = -Math.PI / 2;
scene.add(rosRoot);

let robot = null;
let follow = true;
controls.addEventListener("start", () => {
  follow = false;
});

async function loadUrdf() {
  const manager = new THREE.LoadingManager();
  const loader = new URDFLoader(manager);
  loader.parseCollision = false;
  loader.parseVisual = true;

  robot = await loader.loadAsync("/assets/urdf/fw_mid.urdf");
  robot.traverse((c) => {
    if (c.isMesh) {
      c.castShadow = true;
      c.receiveShadow = true;
      if (c.material) {
        c.material.metalness = 0.25;
        c.material.roughness = 0.55;
      }
    }
  });
  for (const j of Object.values(robot.joints || {})) {
    j.ignoreLimits = true;
  }
  rosRoot.add(robot);
  console.log("[urdf] joints", Object.keys(robot.joints || {}));
}

function resize() {
  const w = canvas.clientWidth || window.innerWidth;
  const h = canvas.clientHeight || window.innerHeight - 60;
  renderer.setSize(w, h, false);
  camera.aspect = w / Math.max(h, 1);
  camera.updateProjectionMatrix();
}
window.addEventListener("resize", resize);

function setText(id, value, warn = false) {
  const el = document.getElementById(id);
  if (!el) return;
  el.textContent = value;
  el.classList.toggle("warn", !!warn);
}
function fmt(n, digits = 3, unit = "") {
  if (n === null || n === undefined || Number.isNaN(n)) return "—";
  return `${Number(n).toFixed(digits)}${unit}`;
}
function yn(v) {
  return v ? "是" : "否";
}

function updateBatteryHud(soc, charging, voltage) {
  const widget = document.getElementById("battWidget");
  const pctEl = document.getElementById("battPct");
  const levelEl = document.getElementById("battLevel");
  const boltEl = document.getElementById("battBolt");
  if (!widget || !pctEl || !levelEl) return;

  const known = soc !== null && soc !== undefined && !Number.isNaN(Number(soc));
  const pct = known ? Math.max(0, Math.min(100, Math.round(Number(soc)))) : null;
  const isCharging = !!charging;

  widget.classList.remove("low", "mid", "charging", "unknown");
  if (!known) {
    widget.classList.add("unknown");
    pctEl.textContent = "—%";
    levelEl.style.width = "0%";
    widget.title = "底盘电量：未知";
    if (boltEl) boltEl.hidden = true;
    return;
  }

  pctEl.textContent = `${pct}%`;
  levelEl.style.width = `${pct}%`;
  if (isCharging) widget.classList.add("charging");
  else if (pct <= 20) widget.classList.add("low");
  else if (pct <= 40) widget.classList.add("mid");

  if (boltEl) boltEl.hidden = !isCharging;
  const volt =
    voltage === null || voltage === undefined || Number.isNaN(Number(voltage))
      ? ""
      : ` · ${Number(voltage).toFixed(1)} V`;
  widget.title = `底盘电量 ${pct}%${isCharging ? "（充电中）" : ""}${volt}`;
}

function setBit(id, on, label) {
  const el = document.getElementById(id);
  if (!el) return;
  el.textContent = `${label} ${on ? "开" : "关"}`;
  el.classList.toggle("on", !!on);
  el.classList.toggle("off", !on);
}

function updateMotorEnableHud(motor, online = true) {
  const wheels = ["lf", "rf", "lr", "rr"];
  const drive = motor?.drive || {};
  const steer = motor?.steer || {};
  let driveOn = 0;
  let steerOn = 0;
  for (const w of wheels) {
    const d = !!drive[w];
    const s = !!steer[w];
    if (d) driveOn += 1;
    if (s) steerOn += 1;
    setBit(`en_drive_${w}`, d, "驱动");
    setBit(`en_steer_${w}`, s, "转向");
    const card = document.querySelector(`.wheel-en[data-wheel="${w}"]`);
    if (card) {
      card.classList.toggle("ok", online && d && s);
      card.classList.toggle("bad", online && (!d || !s));
    }
  }
  const allOk = online && driveOn === 4 && steerOn === 4;
  const none = !online || (driveOn === 0 && steerOn === 0);
  let summary;
  if (!online) summary = "离线/未知";
  else if (allOk) summary = "四轮正常使能";
  else if (none) summary = "全部失能";
  else summary = "部分失能";

  const tag = document.getElementById("motorAllTag");
  const hint = document.getElementById("motorAllHint");
  const badge = document.getElementById("motorBadge");
  const banner = document.getElementById("motorBanner");
  if (tag) {
    tag.textContent = summary;
    tag.className = `tag ${allOk ? "" : "bad"}`.trim();
  }
  if (hint) {
    hint.textContent = online
      ? `驱动 ${driveOn}/4 · 转向 ${steerOn}/4`
      : "等待 CAN 电机反馈帧…";
  }
  if (badge) {
    badge.textContent = `使能：${summary}`;
    badge.className = `live motor-live ${allOk ? "ok" : "bad"}`;
  }
  if (banner) {
    banner.textContent = online
      ? `电机使能：${summary}（驱动 ${driveOn}/4 · 转向 ${steerOn}/4）`
      : "电机使能：等待 CAN 反馈…";
    banner.className = `motor-banner ${allOk ? "ok" : "bad"}`;
  }
}

function applyJoints(joints) {
  if (!robot || !joints) return;
  robot.setJointValues(joints);
}

async function poll() {
  try {
    const res = await fetch("/api/state", { cache: "no-store" });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    const s = await res.json();
    const fresh = s.can_ok && s.data_age_s !== null && s.data_age_s < 1.5;
    badge.textContent = fresh
      ? `${s.can_iface} · ${s.msg_hz} Hz · Gazebo URDF`
      : s.can_ok
        ? "CAN 无新数据"
        : "底盘离线（无反馈）";
    badge.className = `live ${fresh ? "ok" : "bad"}`;
    if (!fresh && cmdStatus && !String(cmdStatus.textContent || "").includes("急停")) {
      cmdStatus.textContent = "底盘无 CAN 反馈：使能不会生效。请查车电/急停旋钮/CAN 线";
    }

    setText("gear", s.gear_name || "—");
    setText("estop", s.estop ? "按下" : "释放", !!s.estop);
    setText("error", s.error_name || "—", (s.error_level || 0) !== 0);
    setText(
      "joypad",
      s.io?.joypad_online ? (s.io.joypad_ctrl ? "在线·控制中" : "在线") : "离线"
    );
    setText("charge", yn(s.io?.charge));
    setText("unlock", yn(s.io?.unlock));
    setText("vx", fmt(s.vx, 3, " m/s"));
    setText("vy", fmt(s.vy, 3, " m/s"));
    setText("wz", fmt(s.wz_deg, 2, " °/s"));
    setText("vsteer", fmt(s.steer_virtual_vel, 3, " m/s"));
    setText("asteer", fmt(s.steer_virtual_angle, 1, " °"));
    setText("px", fmt(s.pose?.x, 3, " m"));
    setText("py", fmt(s.pose?.y, 3, " m"));
    setText("pyaw", fmt(((s.pose?.yaw || 0) * 180) / Math.PI, 1, " °"));
    setText("steer_lf", fmt(s.steer_deg?.lf, 1));
    setText("steer_rf", fmt(s.steer_deg?.rf, 1));
    setText("steer_lr", fmt(s.steer_deg?.lr, 1));
    setText("steer_rr", fmt(s.steer_deg?.rr, 1));
    const angleLive = document.getElementById("angleLive");
    if (angleLive) {
      const ok =
        s.angle_crc_ok && s.angle_age_s !== null && s.angle_age_s < 0.2 && (s.angle_hz || 0) > 10;
      angleLive.textContent = ok ? `实时 ${s.angle_hz} Hz` : "等待角度帧";
      angleLive.className = `tag ${ok ? "" : "bad"}`.trim();
    }
    const steerNote = document.getElementById("steerNote");
    if (steerNote) {
      const absMax = Math.max(
        ...["lf", "rf", "lr", "rr"].map((k) => Math.abs(s.steer_deg?.[k] || 0))
      );
      steerNote.textContent =
        s.gear === 1 && absMax > 5
          ? "驻车但轮角未回零：这是底盘真实反馈，不是显示卡死"
          : "";
    }
    setText("wvel_lf", fmt(s.wheel_vel?.lf, 3));
    setText("wvel_rf", fmt(s.wheel_vel?.rf, 3));
    setText("wvel_lr", fmt(s.wheel_vel?.lr, 3));
    setText("wvel_rr", fmt(s.wheel_vel?.rr, 3));
    updateMotorEnableHud(s.motor, fresh);
    setText("icur_lf", fmt(s.drive_current?.lf, 1));
    setText("icur_rf", fmt(s.drive_current?.rf, 1));
    setText("icur_lr", fmt(s.drive_current?.lr, 1));
    setText("icur_rr", fmt(s.drive_current?.rr, 1));
    setText("soc", s.soc == null ? "—" : `${s.soc} %`);
    setText("volt", s.voltage == null ? "—" : `${Number(s.voltage).toFixed(2)} V`);
    setText("curr", s.current == null ? "—" : `${Number(s.current).toFixed(2)} A`);
    setText("ah", s.remaining_ah == null ? "—" : `${Number(s.remaining_ah).toFixed(2)} Ah`);
    updateBatteryHud(s.soc, s.io?.charge, s.voltage);
    setText("impact", yn(s.io?.impact), !!s.io?.impact);
    setText("drop", yn(s.io?.drop), !!s.io?.drop);
    setText(
      "lamps",
      `${s.io?.low_beam ? "近光" : "—"} / ${s.io?.brake_lamp ? "刹车" : "—"}`
    );

    applyJoints(s.joints);

    if (robot && s.pose) {
      robot.position.set(s.pose.x || 0, s.pose.y || 0, 0);
      robot.quaternion.setFromAxisAngle(new THREE.Vector3(0, 0, 1), s.pose.yaw || 0);
      if (follow) {
        controls.target.set(s.pose.x || 0, 0.2, -(s.pose.y || 0));
      }
    }

    if (s.cmd && cmdStatus) {
      const c = s.cmd;
      const age = c.cmd_age_s == null ? "—" : `${c.cmd_age_s}s`;
      if (c.mode === "idle") {
        cmdStatus.textContent = `协同：空闲 · TX ${c.tx_ok ? "OK" : "断"}`;
      } else if (c.mode === "estop" || c.soft_estop) {
        cmdStatus.textContent = `⚠ 软件急停中 · 零速+电机失能 · TX ${c.tx_ok ? "OK" : "断"}`;
      } else if (c.mode === "drive") {
        const phase = c.coasting ? "减速中" : "运行";
        const gearHint = Number(c.gear) === 8 ? "横移" : "4T4D";
        cmdStatus.textContent =
          `协同 ${phase}(${gearHint}) · vx=${Number(c.cmd_vx ?? c.vx).toFixed(2)} ` +
          `vy=${Number(c.cmd_vy ?? c.vy).toFixed(2)} ` +
          `wz=${Number(c.cmd_wz ?? c.wz).toFixed(1)} · ${age}`;
      } else if (c.mode === "hold") {
        cmdStatus.textContent = `停车保持转角 · TX ${c.tx_ok ? "OK" : "断"}`;
      } else if (c.mode === "free") {
        cmdStatus.textContent = `单轮调试中 · ${age}`;
      } else {
        cmdStatus.textContent = `指令：${c.mode}`;
      }
      if (c.tx_err) cmdStatus.textContent += ` · ${c.tx_err}`;
    }
  } catch (err) {
    badge.textContent = "服务断开";
    badge.className = "live bad";
    updateMotorEnableHud(null, false);
    updateBatteryHud(null, false, null);
  }
}

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}

/* ——— 整车 / 单轮操控 ——— */
const cmdStatus = document.getElementById("cmdStatus");
const speedRange = document.getElementById("speedRange");
const turnRange = document.getElementById("turnRange");
const speedVal = document.getElementById("speedVal");
const turnVal = document.getElementById("turnVal");
const wVelRange = document.getElementById("wVelRange");
const wAngRange = document.getElementById("wAngRange");
const wVelVal = document.getElementById("wVelVal");
const wAngVal = document.getElementById("wAngVal");

let selectedWheel = "lf";
let holdTimer = null;
let holdAction = null;

function syncSliders() {
  if (speedVal && speedRange) speedVal.textContent = Number(speedRange.value).toFixed(2);
  if (turnVal && turnRange) turnVal.textContent = String(turnRange.value);
  if (wVelVal && wVelRange) wVelVal.textContent = Number(wVelRange.value).toFixed(2);
  if (wAngVal && wAngRange) wAngVal.textContent = String(wAngRange.value);
}
speedRange?.addEventListener("input", syncSliders);
turnRange?.addEventListener("input", syncSliders);
wVelRange?.addEventListener("input", syncSliders);
wAngRange?.addEventListener("input", syncSliders);
syncSliders();

async function postCmd(body) {
  const res = await fetch("/api/cmd", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  const data = await res.json().catch(() => ({}));
  if (!res.ok || data.ok === false) {
    const msg = data.error || `HTTP ${res.status}`;
    if (cmdStatus) cmdStatus.textContent = `指令失败：${msg}`;
    throw new Error(msg);
  }
  return data;
}

function stopHold() {
  if (holdTimer) {
    clearInterval(holdTimer);
    holdTimer = null;
  }
  holdAction = null;
  document.querySelectorAll(".pad-btn.held, #btnWheelHold.held").forEach((el) => {
    el.classList.remove("held");
  });
}

async function startHold(action, btn) {
  stopHold();
  holdAction = action;
  btn?.classList.add("held");
  const tick = async () => {
    try {
      if (action === "wheel_hold") {
        await postCmd({
          action: "wheel",
          wheel: selectedWheel,
          velocity: Number(wVelRange?.value || 0),
          angle: Number(wAngRange?.value || 0),
          gear: 6,
        });
      } else if (action === "straighten") {
        await postCmd({ action: "straighten", gear: 6 });
      } else if (action === "stop") {
        await postCmd({ action: "stop" });
      } else {
        await postCmd({
          action,
          speed: Number(speedRange?.value || 0.1),
          turn: Number(turnRange?.value || 25),
          gear: 6,
        });
      }
    } catch (_) {
      /* status already shown */
    }
  };
  await tick();
  holdTimer = setInterval(tick, 80);
}

function bindHold(el, action) {
  if (!el) return;
  const down = (e) => {
    e.preventDefault();
    startHold(action, el);
  };
  const up = (e) => {
    e.preventDefault();
    const was = holdAction;
    stopHold();
    if (was && was !== "stop") postCmd({ action: "stop" }).catch(() => {});
  };
  el.addEventListener("pointerdown", down);
  el.addEventListener("pointerup", up);
  el.addEventListener("pointerleave", up);
  el.addEventListener("pointercancel", up);
}

document.querySelectorAll(".pad-btn[data-action]").forEach((btn) => {
  const act = btn.getAttribute("data-action");
  if (act === "stop") {
    btn.addEventListener("click", (e) => {
      e.preventDefault();
      stopHold();
      postCmd({ action: "stop" }).catch(() => {});
    });
  } else {
    bindHold(btn, act);
  }
});

document.getElementById("btnStop")?.addEventListener("click", () => {
  stopHold();
  postCmd({ action: "stop" }).catch(() => {});
});
document.getElementById("btnEnable")?.addEventListener("click", () => {
  postCmd({ action: "enable" }).catch(() => {});
});
document.getElementById("btnUnlock")?.addEventListener("click", () => {
  postCmd({ action: "unlock" }).catch(() => {});
});
bindHold(document.getElementById("btnStraighten"), "straighten");
document.getElementById("btnEstop")?.addEventListener("click", (e) => {
  e.preventDefault();
  stopHold();
  postCmd({ action: "estop" }).catch(() => {});
});
document.getElementById("btnEstopClear")?.addEventListener("click", () => {
  postCmd({ action: "estop_clear" }).catch(() => {});
});

document.querySelectorAll(".chip[data-wheel]").forEach((btn) => {
  btn.addEventListener("click", () => {
    document.querySelectorAll(".chip[data-wheel]").forEach((b) => b.classList.remove("active"));
    btn.classList.add("active");
    selectedWheel = btn.getAttribute("data-wheel") || "lf";
  });
});

document.getElementById("btnWheelApply")?.remove();
document.getElementById("btnWheelZero")?.addEventListener("click", () => {
  if (wVelRange) wVelRange.value = "0";
  if (wAngRange) wAngRange.value = "0";
  syncSliders();
  stopHold();
  postCmd({ action: "stop" }).catch(() => {});
});
bindHold(document.getElementById("btnWheelHold"), "wheel_hold");

window.addEventListener("blur", () => {
  stopHold();
  postCmd({ action: "stop" }).catch(() => {});
});
window.addEventListener("keydown", (e) => {
  if (e.code === "Space" || e.key === "Escape") {
    e.preventDefault();
    stopHold();
    postCmd({ action: "estop" }).catch(() => {});
  }
});

resize();
animate();
setInterval(poll, 100);
poll(); // 立刻拉使能状态，不等 URDF
try {
  await loadUrdf();
} catch (err) {
  showErr(`URDF 加载失败: ${err}`);
  console.error(err);
}
