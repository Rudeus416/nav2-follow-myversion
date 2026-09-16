const $ = (id) => document.getElementById(id);
let previewUrl = null, previewBusy = false, initialized = false;
let followValues = {}, motionStatus = { motion_mode: 'auto', following: false, force_stopped: false };
let manualTimer = null, manualDirection = 'stop';
let persistImages = false;
const advanced = {
  detection_confidence: ['检测置信度', 0.01, 0.99, 0.01],
  detection_iou: ['检测 IOU', 0.01, 0.99, 0.01],
  segment_imgsz: ['检测输入尺寸', 320, 1280, 32],
  depth_imgsz: ['深度输入尺寸', 320, 1280, 32],
  kp_distance: ['距离 Kp', 0, 10, 0.01], ki_distance: ['距离 Ki', 0, 10, 0.01],
  kd_distance: ['距离 Kd', 0, 10, 0.01], kp_bearing: ['方向 Kp', 0, 10, 0.01],
  kd_bearing: ['方向 Kd', 0, 10, 0.01],
  max_forward_mps: ['最大前进速度 m/s', 0.01, 2, 0.01],
  max_reverse_mps: ['最大后退速度 m/s', 0, 1, 0.01],
  max_yaw_radps: ['最大角速度 rad/s', 0.01, 3, 0.01],
  max_linear_accel_mps2: ['最大线加速度 m/s²', 0.01, 5, 0.01],
  max_yaw_accel_radps2: ['最大角加速度 rad/s²', 0.01, 10, 0.01],
  distance_deadband_m: ['距离死区 m', 0, 1, 0.01],
  bearing_deadband_deg: ['方向死区 度', 0, 30, 0.1],
  manual_linear_accel_mps2: ['手动线加速度 m/s²', 0.01, 5, 0.01],
  manual_yaw_accel_radps2: ['手动角加速度 rad/s²', 0.01, 10, 0.01],
};
const basic = ['camera_height_m', 'follow_distance_m', 'target_id', 'distance_mode', 'process_fps'];
for (const [name, [label, min, max, step]] of Object.entries(advanced)) {
  const wrapper = document.createElement('label');
  wrapper.textContent = label;
  const input = document.createElement('input');
  Object.assign(input, { id: name, type: 'number', min, max, step, required: true });
  wrapper.append(input);
  $('advancedFields').append(wrapper);
}

async function request(url, options) {
  const response = await fetch(url, { cache: 'no-store', ...options });
  if (!response.ok) {
    let detail = await response.text();
    try { detail = JSON.parse(detail).detail || detail; } catch (_) { /* plain text */ }
    throw new Error(typeof detail === 'string' ? detail : JSON.stringify(detail));
  }
  return response.json();
}
function post(url, body) {
  return request(url, { method: 'POST', ...(body === undefined ? {} : {
    headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body),
  }) });
}
function setConfig(config) {
  $('fps').value = config.capture.fps;
  $('duration').value = config.capture.duration;
  persistImages = Boolean(config.capture.persist_images);
  updateStorageButton();
  $('exposure').value = config.camera.exposure_us;
  $('gain').value = config.camera.gain;
  followValues = { ...config.follow };
  for (const name of [...basic, ...Object.keys(advanced)]) $(name).value = config.follow[name];
  initialized = true;
}
function formConfig() {
  const follow = { ...followValues };
  for (const name of [...basic, ...Object.keys(advanced)]) {
    const element = $(name);
    if (!element.checkValidity()) throw new Error(`参数 ${name} 无效`);
    follow[name] = name === 'distance_mode' ? element.value : Number(element.value);
  }
  return { capture: { fps: Number($('fps').value), duration: Number($('duration').value), persist_images: persistImages },
    camera: { exposure_us: Number($('exposure').value), gain: Number($('gain').value) }, follow };
}
function clearPreview(message) {
  $('noImage').textContent = message;
  $('noImage').classList.remove('hidden');
  $('live').removeAttribute('src');
  if (previewUrl) URL.revokeObjectURL(previewUrl);
  previewUrl = null;
}
function updateStorageButton() {
  $('storageButton').textContent = persistImages ? '存储方式：写入 data' : '存储方式：仅内存';
}
async function refreshPreview() {
  if (previewBusy) return;
  previewBusy = true;
  let newUrl = null;
  try {
    const response = await fetch('/api/snapshot', { cache: 'no-store' });
    if (response.status === 503) { clearPreview('正在等待检测结果…'); return; }
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    newUrl = URL.createObjectURL(await response.blob());
    const image = $('live');
    await new Promise((resolve, reject) => {
      image.onload = resolve;
      image.onerror = () => reject(new Error('图像解码失败'));
      image.src = newUrl;
    });
    if (previewUrl) URL.revokeObjectURL(previewUrl);
    previewUrl = newUrl;
    newUrl = null;
    $('noImage').classList.add('hidden');
  } catch (error) {
    if (newUrl) URL.revokeObjectURL(newUrl);
    clearPreview(`检测结果连接失败：${error.message}`);
  } finally { previewBusy = false; setTimeout(refreshPreview, 500); }
}
function showStatus(s) {
  motionStatus = s;
  const active = s.image_stream_active || s.processed_image_age_seconds !== null && s.processed_image_age_seconds < 5;
  $('connection').textContent = active ? '画面更新中' : '等待图像';
  $('connection').className = `badge ${active ? 'good' : 'bad'}`;
  $('cameraStatus').textContent = ({ managed: '本服务运行中', external: '外部 ROS 节点运行中', stopped: '已停止' })[s.device_mode] || '未知';
  persistImages = Boolean(s.persist_images);
  updateStorageButton();
  $('saveStatus').textContent = !s.camera_process_running ? '等待摄像头' : !s.saving ? '已停止采集'
    : persistImages && s.write_error ? '写盘失败，内存继续' : persistImages ? '采集中并写盘' : '采集中（仅内存）';
  $('saveFps').textContent = `${s.save_fps} 张/秒`;
  $('saveDuration').textContent = s.duration_seconds === 0 ? '不限时' : `${s.duration_seconds} 秒`;
  $('storageStatus').textContent = persistImages ? '内存 + data目录' : '仅进程内存';
  $('imageAge').textContent = s.last_image_age_seconds === null ? '尚未收到' : `${s.last_image_age_seconds} 秒前`;
  $('received').textContent = s.received_total;
  $('saved').textContent = s.saved_total;
  $('persisted').textContent = s.persisted_total;
  $('retained').textContent = s.retained_frames;
  $('dropped').textContent = s.dropped_saves;
  $('diskFree').textContent = `${s.disk_free_mb} MB`;
  $('error').textContent = s.error || s.write_error || s.processing_error || '';
  $('startDevice').disabled = !s.device_control_available || s.device_mode === 'managed';
  $('stopDevice').disabled = !s.device_control_available || s.device_mode !== 'managed';
  $('cameraForm').querySelector('button').disabled = !s.camera_process_running;
  if (s.device_mode === 'external') $('deviceMessage').textContent = '摄像头由其他终端启动；设备启动和停止需在该终端操作。';
  $('modeStatus').textContent = s.force_stopped ? '强制停止' : s.motion_mode === 'manual' ? '手动运行' : s.following ? '自动跟随中' : '自动待机';
  $('targetStatus').textContent = s.target_visible ? `ID ${s.follow_settings.target_id} · ${s.target_distance_m === null ? '距离未知' : `${s.target_distance_m.toFixed(2)} m`}` : `ID ${s.follow_settings.target_id} · 丢失/未出现`;
  $('linearX').textContent = `${Number(s.linear_x_mps).toFixed(3)} m/s`;
  $('angularZ').textContent = `${Number(s.angular_z_radps).toFixed(3)} rad/s`;
  const battery = s.battery_percentage === null || s.battery_percentage === undefined
    ? null : Number(s.battery_percentage);
  const voltage = s.battery_voltage_v === null || s.battery_voltage_v === undefined
    ? null : Number(s.battery_voltage_v);
  $('batteryLevel').textContent = Number.isFinite(battery)
    ? `${battery.toFixed(1)}%${Number.isFinite(voltage) ? ` · ${voltage.toFixed(1)} V` : ''}`
    : '等待反馈';
  $('processedCount').textContent = s.processed_total;
  $('drawnCount').textContent = s.drawn_total;
  $('skippedCount').textContent = s.skipped_processing_frames;
  $('skippedDrawCount').textContent = s.skipped_drawing_frames;
  $('processingLatency').textContent = s.processing_latency_seconds === null ? '—' : `${Number(s.processing_latency_seconds).toFixed(3)} 秒`;
  $('displayLatency').textContent = s.display_latency_seconds === null ? '—' : `${Number(s.display_latency_seconds).toFixed(3)} 秒`;
  $('followButton').textContent = s.following ? '结束跟随' : '开始跟随';
  $('followButton').disabled = s.motion_mode !== 'auto' || s.force_stopped;
  $('modeButton').textContent = s.motion_mode === 'auto' ? '切换为手动运行' : '切换为自动跟随';
  $('releaseStop').hidden = !s.force_stopped;
  for (const button of $('manualPad').querySelectorAll('button')) button.disabled = s.motion_mode !== 'manual' || s.force_stopped;
}
async function refreshStatus() {
  try { showStatus(await request('/api/status')); }
  catch (error) { $('connection').textContent = '服务连接失败'; $('connection').className = 'badge bad'; $('error').textContent = `状态接口不可用：${error.message}`; }
}
async function renderFrames(endpoint, container, empty) {
  const frames = await request(endpoint);
  $(container).replaceChildren();
  for (const frame of frames.slice(0, 10)) {
    const link = document.createElement('a');
    Object.assign(link, { href: frame.url, target: '_blank', rel: 'noopener' });
    const image = document.createElement('img');
    Object.assign(image, { src: frame.url, alt: frame.name, loading: 'lazy' });
    link.append(image); $(container).append(link);
  }
  if (!frames.length) $(container).textContent = empty;
}
async function refreshFrames() {
  try { await Promise.all([renderFrames('/api/frames', 'frames', '还没有保存的照片。'),
    renderFrames('/api/processed-frames', 'processedFrames', '还没有检测结果图片。')]); }
  catch (error) { $('actionMessage').textContent = `照片列表不可用：${error.message}`; }
}
async function refreshConfigFiles(selected) {
  const names = await request('/api/config/files');
  $('configSelect').replaceChildren(new Option('选择配置文件…', ''));
  for (const name of names) $('configSelect').add(new Option(name, name));
  if (selected) $('configSelect').value = selected;
}
async function loadSelectedConfig() {
  const name = $('configSelect').value;
  if (!name) return;
  try {
    const result = await post(`/api/config/load/${encodeURIComponent(name)}`);
    setConfig(result.config);
    await refreshStatus();
    $('configMessage').textContent = `已应用 ${name}`;
  } catch (error) { $('configMessage').textContent = error.message; }
}
async function applyFollow() {
  followValues = await post('/api/follow/settings', formConfig().follow);
  await refreshStatus();
}
async function cameraControl(url, success, body) {
  try { await post(url, body); $('deviceMessage').textContent = success; await refreshStatus(); }
  catch (error) { $('deviceMessage').textContent = error.message; }
}
$('startDevice').addEventListener('click', () => cameraControl('/api/device/start', '正在启动摄像头。'));
$('stopDevice').addEventListener('click', async () => {
  try { await post('/api/follow/stop'); await cameraControl('/api/device/stop', '已停止摄像头和跟随。'); }
  catch (error) { $('deviceMessage').textContent = error.message; }
});
$('cameraForm').addEventListener('submit', (event) => {
  event.preventDefault();
  cameraControl('/api/camera/settings', '已应用曝光与增益。', { exposure_us: Number($('exposure').value), gain: Number($('gain').value) });
});
$('captureForm').addEventListener('submit', async (event) => {
  event.preventDefault();
  try { await post('/api/capture/start', { fps: Number($('fps').value), duration: Number($('duration').value), persist_images: persistImages }); $('actionMessage').textContent = '已按新频率开始采集。'; refreshStatus(); }
  catch (error) { $('actionMessage').textContent = error.message; }
});
$('stopButton').addEventListener('click', async () => {
  try { await post('/api/capture/stop'); await post('/api/follow/stop'); $('actionMessage').textContent = '已停止采集和跟随。'; refreshStatus(); }
  catch (error) { $('actionMessage').textContent = error.message; }
});
$('storageButton').addEventListener('click', async () => {
  try {
    const status = await post(`/api/storage/${!persistImages}`);
    persistImages = Boolean(status.persist_images);
    updateStorageButton();
    $('actionMessage').textContent = persistImages ? '后续原图和结果图会写入 data 目录。' : '后续原图和结果图只保存在进程内存。';
    await refreshStatus();
  } catch (error) { $('actionMessage').textContent = error.message; }
});
$('followForm').addEventListener('submit', async (event) => {
  event.preventDefault();
  try { await applyFollow(); $('motionMessage').textContent = '已应用跟随参数。'; }
  catch (error) { $('motionMessage').textContent = error.message; }
});
$('target_id').addEventListener('change', async () => {
  if (!initialized) return;
  try { await applyFollow(); $('motionMessage').textContent = `已选择人物 ID ${$('target_id').value}。`; }
  catch (error) { $('motionMessage').textContent = error.message; }
});
$('followButton').addEventListener('click', async () => {
  try { if (motionStatus.following) await post('/api/follow/stop');
    else { await applyFollow(); await post('/api/follow/start'); }
    await refreshStatus(); }
  catch (error) { $('motionMessage').textContent = error.message; }
});
$('modeButton').addEventListener('click', async () => {
  stopManual();
  try { await post(`/api/mode/${motionStatus.motion_mode === 'auto' ? 'manual' : 'auto'}`); await refreshStatus(); }
  catch (error) { $('motionMessage').textContent = error.message; }
});
$('forceStop').addEventListener('click', async () => {
  stopManual();
  try { await post('/api/force-stop'); await refreshStatus(); $('motionMessage').textContent = '已强制停止，持续发送零速度。'; }
  catch (error) { $('motionMessage').textContent = error.message; }
});
$('releaseStop').addEventListener('click', async () => {
  try { await post('/api/force-stop/release'); await refreshStatus(); $('motionMessage').textContent = '已解除强制停止。'; }
  catch (error) { $('motionMessage').textContent = error.message; }
});
function sendManual(direction) { post('/api/manual', { direction }).catch((error) => { $('motionMessage').textContent = error.message; stopManual(); }); }
function stopManual() {
  if (manualTimer) clearInterval(manualTimer);
  manualTimer = null;
  const wasMoving = manualDirection !== 'stop';
  manualDirection = 'stop';
  if (wasMoving) sendManual('stop');
}
for (const button of $('manualPad').querySelectorAll('button')) {
  button.addEventListener('pointerdown', (event) => {
    if (button.disabled) return;
    event.preventDefault(); stopManual();
    manualDirection = button.dataset.direction;
    button.setPointerCapture(event.pointerId);
    sendManual(manualDirection);
    manualTimer = setInterval(() => sendManual(manualDirection), 120);
  });
  button.addEventListener('pointerup', stopManual);
  button.addEventListener('pointercancel', stopManual);
  button.addEventListener('lostpointercapture', stopManual);
}
window.addEventListener('blur', stopManual);
document.addEventListener('visibilitychange', () => { if (document.hidden) stopManual(); });
$('saveConfig').addEventListener('click', async () => {
  try { const result = await post('/api/config/save', formConfig()); await refreshConfigFiles(result.name); $('configMessage').textContent = `已保存 ${result.name}`; }
  catch (error) { $('configMessage').textContent = error.message; }
});
$('loadConfig').addEventListener('click', async () => {
  const name = $('configSelect').value;
  if (!name) { $('configMessage').textContent = '请先选择配置文件。'; return; }
  await loadSelectedConfig();
});
$('refreshFrames').addEventListener('click', refreshFrames);
$('refreshProcessed').addEventListener('click', refreshFrames);
window.addEventListener('beforeunload', () => { stopManual(); if (previewUrl) URL.revokeObjectURL(previewUrl); });
(async () => {
  try { setConfig(await request('/api/config/current')); await refreshConfigFiles(); }
  catch (error) { $('configMessage').textContent = `配置初始化失败：${error.message}`; }
  refreshStatus(); refreshPreview(); refreshFrames();
  setInterval(refreshStatus, 1000);
  setInterval(refreshFrames, 10000);
})();
