const $ = (id) => document.getElementById(id);
const controls = { yaw: $('yaw'), pitch: $('pitch'), roll: $('roll'), tx: $('tx'), ty: $('ty'), tz: $('tz') };
const state = {
  values: { yaw: 0, pitch: 0, roll: 0, tx: 0, ty: 0, tz: 0 },
  mode: 'rotate',
  calibration: null,
  colors: [],
  pair: null,
  image: null,
  imageRequest: 0,
  pairs: [],
  boundFile: null,
  drag: null,
  layout: null,
  rangeMax: 2,
  radarView: { yaw: Math.PI / 4, elevation: Math.PI / 6, zoom: 1, drag: null },
  completedCapture: null,
};
let capturePollTimer = null;

async function api(path, options = {}) {
  const response = await fetch(path, options);
  const content = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(content.detail || `${response.status} ${response.statusText}`);
  return content;
}

function status(message, isError = false) {
  $('status').textContent = message;
  $('status').classList.toggle('error', isError);
}

function fillSelect(select, options, emptyText) {
  select.replaceChildren();
  if (!options.length) {
    const option = document.createElement('option');
    option.textContent = emptyText;
    option.value = '';
    select.append(option);
    select.disabled = true;
    return;
  }
  select.disabled = false;
  for (const item of options) {
    const option = document.createElement('option');
    option.value = item.value;
    option.textContent = item.label;
    select.append(option);
  }
}

function multiply(a, b) {
  return a.map((row) => b[0].map((_, column) =>
    row.reduce((sum, value, index) => sum + value * b[index][column], 0)));
}

function rotationAndTranslation() {
  const { yaw, pitch, roll, tx, ty, tz } = state.values;
  const y = yaw * Math.PI / 180;
  const p = pitch * Math.PI / 180;
  const r = roll * Math.PI / 180;
  const rz = [[Math.cos(y), -Math.sin(y), 0], [Math.sin(y), Math.cos(y), 0], [0, 0, 1]];
  const ry = [[Math.cos(p), 0, Math.sin(p)], [0, 1, 0], [-Math.sin(p), 0, Math.cos(p)]];
  const rx = [[1, 0, 0], [0, Math.cos(r), -Math.sin(r)], [0, Math.sin(r), Math.cos(r)]];
  const axes = state.calibration?.radar_to_optical_axes || [[0, -1, 0], [0, 0, -1], [1, 0, 0]];
  const opticalTranslation = axes.map((row) => row[0] * tx + row[1] * ty + row[2] * tz);
  return { rotation: multiply(axes, multiply(multiply(rz, ry), rx)), translation: opticalTranslation };
}

function updateMatrix() {
  const { rotation, translation } = rotationAndTranslation();
  const rows = rotation.map((row, index) => [...row, translation[index]]);
  rows.push([0, 0, 0, 1]);
  $('matrixDisplay').textContent = rows.map((row) =>
    '[ ' + row.map((value) => `${value >= 0 ? ' ' : ''}${value.toFixed(4)}`).join(' ') + ' ]'
  ).join('\n');
}

function project(point, rotation, translation, imageWidth, imageHeight) {
  const p = [0, 1, 2].map((row) =>
    rotation[row][0] * point[0] + rotation[row][1] * point[1]
    + rotation[row][2] * point[2] + translation[row]);
  if (p[2] <= 0.05 || !p.every(Number.isFinite)) return null;
  const k = state.calibration.intrinsics;
  const x = p[0] / p[2], y = p[1] / p[2];
  const [k1, k2, p1, p2, k3] = k.distortion;
  const r2 = x * x + y * y;
  const radial = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
  const xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
  const yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
  const u = (k.fx * xd + k.cx) * imageWidth / k.width;
  const v = (k.fy * yd + k.cy) * imageHeight / k.height;
  if (!Number.isFinite(u) || !Number.isFinite(v) || u < 0 || v < 0
      || u >= imageWidth || v >= imageHeight) return null;
  return [u, v];
}

function pointRange(point) {
  return Number.isFinite(point[3]) && point[3] > 0
    ? point[3] : Math.hypot(point[0], point[1], point[2]);
}

function pointColor(point) {
  const index = Math.max(0, Math.min(255, Math.round(pointRange(point) / state.rangeMax * 255)));
  return state.colors[index] || '#ffca8f';
}

function niceStep(span) {
  const magnitude = 10 ** Math.floor(Math.log10(span));
  const ratio = span / magnitude;
  return (ratio <= 1 ? 1 : ratio <= 2 ? 2 : ratio <= 5 ? 5 : 10) * magnitude;
}

function radarBounds(points) {
  const minimum = [0, 0, 0], maximum = [0, 0, 0];
  for (const point of points) {
    for (let axis = 0; axis < 3; axis++) {
      minimum[axis] = Math.min(minimum[axis], point[axis]);
      maximum[axis] = Math.max(maximum[axis], point[axis]);
    }
  }
  const span = Math.max(1, ...minimum.map((value, axis) => maximum[axis] - value));
  // Reserve visible height for a planar radar frame without changing the point coordinates.
  if (maximum[2] - minimum[2] < span * 0.3) {
    const middle = (minimum[2] + maximum[2]) / 2;
    minimum[2] = middle - span * 0.15;
    maximum[2] = middle + span * 0.15;
  }
  const step = niceStep(span / 5);
  if (maximum[0] - minimum[0] < step) maximum[0] = minimum[0] + step;
  if (maximum[1] - minimum[1] < step) {
    minimum[1] -= step;
    maximum[1] += step;
  }
  return {
    minimum: minimum.map((value) => Math.floor(value / step) * step),
    maximum: maximum.map((value) => Math.ceil(value / step) * step),
    step,
  };
}

function renderRadar() {
  const canvas = $('radarCanvas'), stage = $('radarStage');
  const width = stage.clientWidth, height = stage.clientHeight;
  if (!width || !height) return;
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  canvas.width = Math.round(width * dpr);
  canvas.height = Math.round(height * dpr);
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, width, height);
  const points = (state.pair?.points || []).filter((point) =>
    point.length >= 3 && point.slice(0, 3).every(Number.isFinite));
  $('radarPointCount').textContent = state.pair ? `${points.length} 个雷达点` : '等待数据';
  const zExtent = points.reduce(([low, high], point) =>
    [Math.min(low, point[2]), Math.max(high, point[2])], [Infinity, -Infinity]);
  $('radarDimensionNote').textContent = points.length && zExtent[1] - zExtent[0] < 1e-5
    ? `本配对所有检测点的 Z 坐标均为 ${points[0][2].toFixed(3)} m；它们实际位于同一平面。`
    : '';

  const { minimum, maximum, step } = radarBounds(points);
  const middle = minimum.map((value, axis) => (value + maximum[axis]) / 2);
  const { yaw, elevation, zoom } = state.radarView;
  const cosYaw = Math.cos(yaw), sinYaw = Math.sin(yaw);
  const cosElevation = Math.cos(elevation), sinElevation = Math.sin(elevation);
  const rotate = (point) => {
    const x = point[0] - middle[0], y = point[1] - middle[1], z = point[2] - middle[2];
    const horizontal = cosYaw * x - sinYaw * y;
    const alongView = sinYaw * x + cosYaw * y;
    return [horizontal, sinElevation * alongView + cosElevation * z,
      cosElevation * alongView - sinElevation * z];
  };
  const corners = [];
  for (const x of [minimum[0], maximum[0]]) {
    for (const y of [minimum[1], maximum[1]]) {
      for (const z of [minimum[2], maximum[2]]) corners.push([x, y, z]);
    }
  }
  const projectedCorners = corners.map(rotate);
  const minU = Math.min(...projectedCorners.map((point) => point[0]));
  const maxU = Math.max(...projectedCorners.map((point) => point[0]));
  const minV = Math.min(...projectedCorners.map((point) => point[1]));
  const maxV = Math.max(...projectedCorners.map((point) => point[1]));
  const margin = Math.min(54, Math.min(width, height) * 0.14);
  const scale = Math.min((width - 2 * margin) / Math.max(maxU - minU, 0.01),
    (height - 2 * margin) / Math.max(maxV - minV, 0.01)) * zoom;
  const toScreen = (point) => {
    const [u, v, depth] = rotate(point);
    return [width / 2 + (u - (minU + maxU) / 2) * scale,
      height / 2 - (v - (minV + maxV) / 2) * scale, depth];
  };
  const drawLine = (start, end, color, lineWidth = 1) => {
    const a = toScreen(start), b = toScreen(end);
    ctx.beginPath();
    ctx.moveTo(a[0], a[1]);
    ctx.lineTo(b[0], b[1]);
    ctx.strokeStyle = color;
    ctx.lineWidth = lineWidth;
    ctx.stroke();
  };

  // A wireframe volume and the XY plane make the radar's depth and scale visible.
  for (let axis = 0; axis < 3; axis++) {
    const others = [0, 1, 2].filter((index) => index !== axis);
    for (const sideA of [minimum[others[0]], maximum[others[0]]]) {
      for (const sideB of [minimum[others[1]], maximum[others[1]]]) {
        const start = [0, 0, 0], end = [0, 0, 0];
        start[axis] = minimum[axis]; end[axis] = maximum[axis];
        start[others[0]] = end[others[0]] = sideA;
        start[others[1]] = end[others[1]] = sideB;
        drawLine(start, end, '#4b505d88');
      }
    }
  }
  const ticks = (axis) => {
    const first = Math.ceil(minimum[axis] / step - 1e-8);
    const last = Math.floor(maximum[axis] / step + 1e-8);
    return Array.from({ length: last - first + 1 }, (_, index) => (first + index) * step);
  };
  for (const x of ticks(0)) drawLine([x, minimum[1], 0], [x, maximum[1], 0], '#60657155');
  for (const y of ticks(1)) drawLine([minimum[0], y, 0], [maximum[0], y, 0], '#60657155');

  const axisColors = ['#f58c84', '#97d9af', '#91baff'];
  const axisNames = ['X 前 (m)', 'Y 左 (m)', 'Z 上 (m)'];
  ctx.font = '11px Inter, system-ui, sans-serif';
  ctx.textBaseline = 'middle';
  const drawLabel = (label, x, y, color) => {
    ctx.lineWidth = 3;
    ctx.strokeStyle = '#111318';
    ctx.strokeText(label, x, y);
    ctx.fillStyle = color;
    ctx.fillText(label, x, y);
  };
  for (let axis = 0; axis < 3; axis++) {
    const start = [0, 0, 0], end = [0, 0, 0];
    start[axis] = minimum[axis]; end[axis] = maximum[axis];
    drawLine(start, end, axisColors[axis], 1.8);
    for (const value of ticks(axis)) {
      if (Math.abs(value) < step * 0.001) continue;
      const position = [0, 0, 0]; position[axis] = value;
      const p = toScreen(position);
      ctx.beginPath();
      ctx.arc(p[0], p[1], 2.2, 0, Math.PI * 2);
      ctx.fillStyle = axisColors[axis];
      ctx.fill();
      const label = Number(value.toFixed(3)).toString();
      drawLabel(label, p[0] + (axis === 1 ? -18 : 5), p[1] + (axis === 2 ? -8 : 13), '#b7bac5');
    }
    const endpoint = toScreen(end);
    drawLabel(axisNames[axis], endpoint[0] + 6, endpoint[1] - 11, axisColors[axis]);
  }
  const origin = toScreen([0, 0, 0]);
  drawLabel('0', origin[0] + 5, origin[1] + 13, '#e4e2e7');

  points.map((point) => ({ point, screen: toScreen(point) }))
    .sort((a, b) => b.screen[2] - a.screen[2])
    .forEach(({ point, screen }) => {
      ctx.beginPath();
      ctx.arc(screen[0], screen[1], 4.5, 0, Math.PI * 2);
      ctx.fillStyle = pointColor(point);
      ctx.fill();
      ctx.lineWidth = 1.2;
      ctx.strokeStyle = '#111218';
      ctx.stroke();
    });
}

function render() {
  updateMatrix();
  renderRadar();
  const canvas = $('projectionCanvas');
  const stage = $('canvasStage');
  const width = stage.clientWidth, height = stage.clientHeight;
  if (!width || !height) return;
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  canvas.width = Math.round(width * dpr);
  canvas.height = Math.round(height * dpr);
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, width, height);
  if (!state.image || !state.pair || !state.calibration) {
    $('visibleCount').textContent = '等待数据';
    return;
  }
  const imageWidth = state.image.naturalWidth, imageHeight = state.image.naturalHeight;
  const scale = Math.min(width / imageWidth, height / imageHeight);
  const left = (width - imageWidth * scale) / 2;
  const top = (height - imageHeight * scale) / 2;
  state.layout = { left, top, scale, imageWidth, imageHeight };
  ctx.drawImage(state.image, left, top, imageWidth * scale, imageHeight * scale);
  const { rotation, translation } = rotationAndTranslation();
  let visible = 0;
  for (const point of state.pair.points) {
    const pixel = project(point, rotation, translation, imageWidth, imageHeight);
    if (!pixel) continue;
    const x = left + pixel[0] * scale, y = top + pixel[1] * scale;
    ctx.beginPath();
    ctx.arc(x, y, 5, 0, Math.PI * 2);
    ctx.fillStyle = pointColor(point);
    ctx.fill();
    ctx.lineWidth = 1.5;
    ctx.strokeStyle = '#0a0a0c';
    ctx.stroke();
    visible++;
  }
  $('visibleCount').textContent = `${visible} / ${state.pair.points.length} 点落在图像内`;
}

function syncInputs() {
  for (const [key, control] of Object.entries(controls)) {
    control.value = Number(state.values[key].toFixed(key.startsWith('t') ? 4 : 3));
  }
  render();
}

async function loadSessions(preferredSession = null, chooseLatest = false) {
  const previousSession = $('sessionSelect').value;
  const previousPair = $('pairSelect').value;
  const sessions = await api('/api/sessions');
  fillSelect($('sessionSelect'), sessions.map((session) => ({
    value: session.name,
    label: `${session.name} · ${session.pair_count} 组`,
  })), '暂无采集批次');
  if (sessions.length) {
    const names = sessions.map((session) => session.name);
    const selected = names.includes(preferredSession) ? preferredSession
      : !chooseLatest && names.includes(previousSession) ? previousSession : names[0];
    $('sessionSelect').value = selected;
    await loadPairs(selected === previousSession ? previousPair : null);
  } else {
    state.pair = null;
    state.image = null;
    fillSelect($('pairSelect'), [], '暂无配对');
    render();
    status('暂无标定数据。可在页面下方启动一批采集。');
  }
}

async function loadPairs(preferredPair = null) {
  const request = ++state.imageRequest;
  state.pair = null;
  state.image = null;
  state.layout = null;
  const name = $('sessionSelect').value;
  render();
  const pairs = name ? await api(`/api/sessions/${encodeURIComponent(name)}/pairs`) : [];
  if (request !== state.imageRequest || name !== $('sessionSelect').value) return;
  state.pairs = pairs;
  fillSelect($('pairSelect'), state.pairs.map((pair) => ({
    value: pair.id,
    label: `#${pair.id} · Δt ${pair.abs_delta_ms.toFixed(2)} ms`,
  })), '暂无配对');
  if (preferredPair && state.pairs.some((pair) => pair.id === preferredPair)) {
    $('pairSelect').value = preferredPair;
  }
  if (state.pairs.length) await loadPair();
  else {
    $('pairInfo').textContent = '本次采集没有落入时间阈值的图像—雷达配对。';
    render();
  }
}

async function loadPair() {
  const session = $('sessionSelect').value, pairId = $('pairSelect').value;
  if (!session || !pairId) return;
  const request = ++state.imageRequest;
  state.pair = null;
  state.image = null;
  state.layout = null;
  render();
  const pair = await api(`/api/sessions/${encodeURIComponent(session)}/pairs/${encodeURIComponent(pairId)}`);
  if (request !== state.imageRequest) return;
  state.pair = pair;
  const ranges = pair.points.map((point) => pointRange(point)).filter((value) => Number.isFinite(value) && value > 0);
  state.rangeMax = Math.max(2, Math.ceil(Math.max(0, ...ranges)));
  $('legendMax').textContent = `远 · ${state.rangeMax} m`;
  $('radarLegendMax').textContent = `远 · ${state.rangeMax} m`;
  const cameraTime = new Date(Number(pair.camera_stamp_ns) / 1e6).toLocaleString();
  $('pairInfo').innerHTML = `相机：${cameraTime}<br>雷达时间差：${pair.delta_ns / 1e6 >= 0 ? '+' : ''}${(pair.delta_ns / 1e6).toFixed(3)} ms<br>雷达检测点：${pair.points.length}`;
  renderRadar();
  const image = new Image();
  image.onload = () => {
    if (request !== state.imageRequest) return;
    state.image = image;
    status(`已加载 ${session} / 配对 #${pairId}`);
    render();
  };
  image.onerror = () => status('图像文件加载失败。', true);
  image.src = pair.image_url;
}

function movePair(offset) {
  const select = $('pairSelect');
  const index = Math.max(0, Math.min(select.options.length - 1, select.selectedIndex + offset));
  if (index !== select.selectedIndex) {
    select.selectedIndex = index;
    loadPair().catch((error) => status(error.message, true));
  }
}

function setMode(mode) {
  state.mode = mode;
  for (const [name, button] of [['rotate', $('rotateMode')], ['translate', $('translateMode')]]) {
    const selected = mode === name;
    button.classList.toggle('active', selected);
    button.setAttribute('aria-pressed', String(selected));
  }
  $('gestureHelp').textContent = mode === 'rotate'
    ? '按住左键拖动：左右调绕雷达 Z 轴偏航，上下调绕 Y 轴俯仰；按 Shift 左右拖动，调绕 X 轴横滚。'
    : '按住左键拖动：左右调雷达 Y 轴平移，上下调 Z 轴平移；按 Shift 上下拖动，调 X 轴前后平移。';
}

function onPointerMove(event) {
  if (!state.drag || !state.layout) return;
  const dx = event.clientX - state.drag.x, dy = event.clientY - state.drag.y;
  state.drag = { x: event.clientX, y: event.clientY };
  if (state.mode === 'rotate') {
    if (event.shiftKey) state.values.roll += dx * 0.08;
    else {
      state.values.yaw -= dx * 0.08;
      state.values.pitch += dy * 0.08;
    }
  } else {
    const ranges = state.pair.points.map((point) => point[3]).filter((value) => Number.isFinite(value) && value > 0);
    const depth = ranges.length ? ranges.sort((a, b) => a - b)[Math.floor(ranges.length / 2)] : 2;
    const k = state.calibration.intrinsics;
    const fxDisplay = k.fx * state.layout.imageWidth / k.width * state.layout.scale;
    const fyDisplay = k.fy * state.layout.imageHeight / k.height * state.layout.scale;
    if (event.shiftKey) state.values.tx -= dy * depth / fyDisplay;
    else {
      state.values.ty -= dx * depth / fxDisplay;
      state.values.tz -= dy * depth / fyDisplay;
    }
  }
  syncInputs();
}

function transformPayload() {
  return {
    parameter_frame: 'radar_ros',
    yaw_deg: state.values.yaw, pitch_deg: state.values.pitch, roll_deg: state.values.roll,
    tx_m: state.values.tx, ty_m: state.values.ty, tz_m: state.values.tz,
    session: $('sessionSelect').value || null,
    pair_id: $('pairSelect').value || null,
  };
}

async function refreshTransforms() {
  const filenames = await api('/api/transforms');
  fillSelect($('transformSelect'), filenames.map((name) => ({ value: name, label: name })), '暂无参数文件');
  if (state.boundFile && filenames.includes(state.boundFile)) $('transformSelect').value = state.boundFile;
}

async function loadTransform() {
  const name = $('transformSelect').value;
  if (!name) return;
  const data = await api(`/api/transforms/${encodeURIComponent(name)}`);
  const angles = data.adjustment_euler_deg, translation = data.translation_m;
  if (data.parameter_frame === 'radar_ros' && data.schema_version >= 2) {
    state.values = {
      yaw: Number(angles.yaw), pitch: Number(angles.pitch), roll: Number(angles.roll),
      tx: Number(translation.x), ty: Number(translation.y), tz: Number(translation.z),
    };
  } else if (data.schema_version === 1) {
    // V1 adjustments and translation were expressed in camera optical axes.
    // This conversion leaves the complete radar-to-camera matrix unchanged.
    state.values = {
      yaw: -Number(angles.yaw), pitch: -Number(angles.pitch), roll: Number(angles.roll),
      tx: Number(translation.z), ty: -Number(translation.x), tz: -Number(translation.y),
    };
  } else {
    throw new Error('参数文件的坐标系或版本无法识别。');
  }
  if (!Object.values(state.values).every(Number.isFinite)) throw new Error('参数文件包含无效外参。');
  state.boundFile = name;
  $('activeFile').textContent = `当前修改：${name}`;
  syncInputs();
  status(`已加载外参 ${name}${data.schema_version === 1 ? '；已将旧版相机坐标参数换算为雷达坐标参数' : ''}，后续保存会更新该文件。`);
}

async function saveTransform(asNew = false) {
  const name = asNew ? null : state.boundFile;
  const data = await api(name ? `/api/transforms/${encodeURIComponent(name)}` : '/api/transforms', {
    method: name ? 'PUT' : 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(transformPayload()),
  });
  state.boundFile = data.name;
  $('activeFile').textContent = `当前修改：${data.name}`;
  await refreshTransforms();
  status(`${name ? '已更新' : '已保存'}外参：data/data_transfer/${data.name}`);
}

function captureMessage(message, isError = false) {
  $('captureMessage').textContent = message;
  $('captureMessage').classList.toggle('error', isError);
}

function applyCaptureDefaults(defaults) {
  const form = $('captureForm');
  for (const [name, value] of Object.entries(defaults)) {
    const control = form.elements.namedItem(name);
    if (!control) continue;
    if (control.type === 'checkbox') control.checked = Boolean(value);
    else control.value = value ?? '';
  }
  syncRadarFields();
}

function syncRadarFields() {
  const external = $('captureForm').elements.namedItem('external_radar').checked;
  for (const control of $('captureForm').querySelectorAll('[data-radar-owned]')) {
    control.disabled = external;
  }
}

function capturePayload() {
  const payload = {};
  for (const control of $('captureForm').elements) {
    if (!control.name) continue;
    if (control.disabled && control.hasAttribute('data-radar-owned')) continue;
    if (control.type === 'checkbox') payload[control.name] = control.checked;
    else if (control.type === 'number') payload[control.name] = Number(control.value);
    else payload[control.name] = control.value.trim() || (control.name === 'name' ? null : '');
  }
  return payload;
}

function renderCaptureStatus(job) {
  const titles = {
    idle: '未启动', running: '运行中', stopping: '正在停止',
    completed: '采集完成', stopped: '已停止', failed: '采集失败',
  };
  $('captureState').textContent = titles[job.state] || job.state;
  $('captureState').classList.toggle('capture-failed', job.state === 'failed');
  $('startCapture').disabled = Boolean(job.running || job.state === 'stopping');
  $('stopCapture').disabled = !job.running || job.state === 'stopping';
  $('captureLog').textContent = (job.output || []).join('\n');
  $('captureLog').scrollTop = $('captureLog').scrollHeight;
  if (job.state === 'running') captureMessage('采集进程已启动，正在等待传感器或采集数据。');
  else if (job.state === 'stopping') captureMessage('正在停止采集并保存已收到的数据。');
  else if (job.state === 'failed') captureMessage(`采集失败（退出码 ${job.exit_code}），请查看下方日志。`, true);
  else if (job.state === 'idle') captureMessage('填写参数后可直接启动 capture.py。');
}

function ensureCapturePolling() {
  if (!capturePollTimer) {
    capturePollTimer = window.setInterval(() => {
      refreshCaptureStatus().catch((error) => captureMessage(error.message, true));
    }, 1200);
  }
}

async function refreshCaptureStatus() {
  const job = await api('/api/capture');
  renderCaptureStatus(job);
  if (job.state === 'running' || job.state === 'stopping') {
    ensureCapturePolling();
  } else {
    if (capturePollTimer) window.clearInterval(capturePollTimer);
    capturePollTimer = null;
    if (job.run_id && job.run_id !== state.completedCapture &&
        ['completed', 'stopped'].includes(job.state)) {
      state.completedCapture = job.run_id;
      await loadSessions(job.session_name, true);
      captureMessage(job.state === 'completed'
        ? '采集完成；批次列表已刷新。' : '采集已停止；已刷新可用批次。');
    }
  }
}

async function startCapture(event) {
  event.preventDefault();
  const form = $('captureForm');
  if (!form.reportValidity()) return;
  $('startCapture').disabled = true;
  captureMessage('正在启动采集进程…');
  try {
    const job = await api('/api/capture', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(capturePayload()),
    });
    renderCaptureStatus(job);
    ensureCapturePolling();
  } catch (error) {
    captureMessage(error.message, true);
    $('startCapture').disabled = false;
  }
}

async function stopCapture() {
  const job = await api('/api/capture/stop', { method: 'POST' });
  renderCaptureStatus(job);
  ensureCapturePolling();
}

function bindEvents() {
  $('sessionSelect').addEventListener('change', () => loadPairs().catch((error) => status(error.message, true)));
  $('refreshSessions').addEventListener('click', () =>
    loadSessions().then(() => captureMessage('采集批次列表已刷新。'))
      .catch((error) => status(error.message, true)));
  $('pairSelect').addEventListener('change', () => loadPair().catch((error) => status(error.message, true)));
  $('previousPair').addEventListener('click', () => movePair(-1));
  $('nextPair').addEventListener('click', () => movePair(1));
  $('rotateMode').addEventListener('click', () => setMode('rotate'));
  $('translateMode').addEventListener('click', () => setMode('translate'));
  for (const [key, control] of Object.entries(controls)) {
    control.addEventListener('input', () => {
      const number = Number.parseFloat(control.value);
      if (!Number.isFinite(number)) return;
      state.values[key] = number;
      render();
    });
  }
  $('resetParams').addEventListener('click', () => {
    state.values = { yaw: 0, pitch: 0, roll: 0, tx: 0, ty: 0, tz: 0 };
    syncInputs();
  });
  $('loadTransform').addEventListener('click', () => loadTransform().catch((error) => status(error.message, true)));
  $('saveTransform').addEventListener('click', () => saveTransform().catch((error) => status(error.message, true)));
  $('saveAsNew').addEventListener('click', () => saveTransform(true).catch((error) => status(error.message, true)));
  $('captureForm').addEventListener('submit', startCapture);
  $('captureForm').elements.namedItem('external_radar').addEventListener('change', syncRadarFields);
  $('stopCapture').addEventListener('click', () =>
    stopCapture().catch((error) => captureMessage(error.message, true)));

  const canvas = $('projectionCanvas');
  canvas.addEventListener('pointerdown', (event) => {
    if (event.button !== 0 || !state.image) return;
    state.drag = { x: event.clientX, y: event.clientY };
    canvas.setPointerCapture(event.pointerId);
    canvas.classList.add('dragging');
  });
  canvas.addEventListener('pointermove', onPointerMove);
  const endDrag = () => { state.drag = null; canvas.classList.remove('dragging'); };
  canvas.addEventListener('pointerup', endDrag);
  canvas.addEventListener('pointercancel', endDrag);
  new ResizeObserver(render).observe($('canvasStage'));

  const radarCanvas = $('radarCanvas');
  radarCanvas.addEventListener('pointerdown', (event) => {
    if (event.button !== 0 || !state.pair) return;
    state.radarView.drag = { id: event.pointerId, x: event.clientX, y: event.clientY };
    radarCanvas.setPointerCapture(event.pointerId);
    radarCanvas.classList.add('dragging');
  });
  radarCanvas.addEventListener('pointermove', (event) => {
    const drag = state.radarView.drag;
    if (!drag || drag.id !== event.pointerId) return;
    state.radarView.yaw += (event.clientX - drag.x) * 0.009;
    state.radarView.elevation = Math.max(-1.45, Math.min(1.45,
      state.radarView.elevation - (event.clientY - drag.y) * 0.009));
    drag.x = event.clientX;
    drag.y = event.clientY;
    renderRadar();
  });
  const endRadarDrag = (event) => {
    if (state.radarView.drag?.id !== event.pointerId) return;
    state.radarView.drag = null;
    radarCanvas.classList.remove('dragging');
  };
  radarCanvas.addEventListener('pointerup', endRadarDrag);
  radarCanvas.addEventListener('pointercancel', endRadarDrag);
  radarCanvas.addEventListener('wheel', (event) => {
    if (!state.pair) return;
    event.preventDefault();
    state.radarView.zoom = Math.max(0.5, Math.min(3,
      state.radarView.zoom * Math.exp(-event.deltaY * 0.001)));
    renderRadar();
  }, { passive: false });
  radarCanvas.addEventListener('dblclick', () => {
    state.radarView.yaw = Math.PI / 4;
    state.radarView.elevation = Math.PI / 6;
    state.radarView.zoom = 1;
    renderRadar();
  });
  new ResizeObserver(renderRadar).observe($('radarStage'));
}

async function main() {
  bindEvents();
  const [calibration, colormap] = await Promise.all([
    api('/api/calibration'), api('/api/colormap'),
  ]);
  state.calibration = calibration;
  if (calibration.parameter_frame !== 'radar_ros') {
    $('saveTransform').disabled = true;
    $('saveAsNew').disabled = true;
    throw new Error('标定服务仍运行旧版坐标约定，请重启 calibration_server.py 并刷新页面。');
  }
  state.colors = colormap.colors;
  const gradient = `linear-gradient(to right, ${state.colors.filter((_, index) => index % 16 === 0 || index === 255).join(', ')})`;
  $('colorbar').style.background = gradient;
  $('radarColorbar').style.background = gradient;
  await Promise.all([loadSessions(), refreshTransforms()]);
  try {
    applyCaptureDefaults(await api('/api/capture/defaults'));
    await refreshCaptureStatus();
  } catch (error) {
    $('startCapture').disabled = true;
    captureMessage(`网页采集接口不可用，请重启 calibration_server.py：${error.message}`, true);
  }
  render();
}

main().catch((error) => status(error.message, true));
