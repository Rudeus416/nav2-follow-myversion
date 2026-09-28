// 【内容标注 / R27 R28 R29】仅整理本项目新增的 Nav2 面板：导航预览、人物/选点操作和设置。
// 所有可搬移节点只从原 .obstacle-map-panel 内取得；不访问或移动原相机、强停、手动控件。
// 保留原 ID、表单、监听器、hidden/disabled 与图片 URL；不新增控制请求。
(() => {
  'use strict';
  const panel = document.querySelector('.obstacle-map-panel');
  if (!panel || panel.dataset.nav2Layout === 'ready') return;
  // 【范围约束】只缓存自有面板里的节点，组装临时脱离文档时仍使用这些引用。
  const originals = new Map(Array.from(panel.querySelectorAll('[id]'), element => [element.id, element]));
  const el = id => originals.get(id);
  const mapView = panel.querySelector('.obstacle-map-view');
  const wallView = panel.querySelector('.virtual-wall-view');
  const oldLayout = panel.querySelector('.nav-wall-layout');
  if (!mapView || !wallView || !oldLayout || !el('nav2PersonPanel')) return;
  const label = id => el(id)?.closest('label');
  const heading = panel.querySelector('.section-heading');
  const legend = heading?.nextElementSibling;
  const targetHint = el('navTargetMessage')?.nextElementSibling;
  const inflationHint = label('mapShowInflation')?.nextElementSibling;
  const possibleCalibration = el('navVisionToggleMessage')?.nextElementSibling;
  const calibration = possibleCalibration?.querySelector('[data-capture]') ? possibleCalibration : null;
  const observers = [];
  const pointPanel = el('pointPreviewPanel');
  const editor = el('mapEditor');
  panel.id = 'nav2Panel';

  function node(tag, className, text) {
    const element = document.createElement(tag);
    if (className) element.className = className;
    if (text) element.textContent = text;
    return element;
  }
  function move(parent, ...items) {
    for (const item of items) {
      const element = typeof item === 'string' ? el(item) : item;
      if (element) parent.append(element);
    }
    return parent;
  }
  function title(name, hint) {
    const header = node('div', 'nav2-heading');
    header.append(node('h3', '', name));
    if (hint) header.append(node('p', 'hint', hint));
    return header;
  }
  function details(name, id) {
    const card = node('details', 'nav2-details');
    if (id) card.id = id;
    const body = node('div', 'nav2-details-body');
    card.append(node('summary', '', name), body);
    return {card, body};
  }
  function check(id, text) {
    const parent = label(id);
    if (!parent) return null;
    parent.classList.add('nav2-check');
    // 保留 label 与末尾文字节点；缓冲设置脚本依赖这一结构。
    if (text) for (const child of parent.childNodes) {
      if (child.nodeType === Node.TEXT_NODE && child.textContent.trim()) child.textContent = ' ' + text;
    }
    return parent;
  }
  function imageBody(image, hint) {
    const body = node('div', 'nav2-feed-body');
    const placeholder = node('p', 'nav2-placeholder', hint);
    body.append(image, placeholder);
    // 原预览模块继续管理 src/hidden 和 Blob URL，只同步等待提示。
    const refresh = () => { placeholder.hidden = !image.hidden && Boolean(image.getAttribute('src')); };
    const observer = new MutationObserver(refresh);
    observer.observe(image, {attributes: true, attributeFilter: ['src', 'hidden']});
    observers.push(observer); refresh();
    return body;
  }

  // 【导航预览】仅集中自有红黄虚拟墙与疑似障碍画面，原实时视频保留原位。
  const media = node('section', 'nav2-media'); media.id = 'nav2Media';
  media.append(title('导航预览', '虚拟墙与疑似障碍画面（停车查看）'));
  const feeds = node('div', 'nav2-feeds');
  const wallParams = el('wallControls');
  const wallBody = imageBody(el('wallImage'), '开启虚拟墙显示后，在强停状态查看叠加画面');
  wallView.classList.add('nav2-feed');
  wallView.prepend(node('h4', 'nav2-feed-title', '红黄虚拟墙'));
  const wallChecks = move(node('div', 'nav2-feed-controls'), check('wallEnabled', '显示红色禁区墙'),
    check('yellowWallEnabled', '显示黄色视觉障碍墙'));
  wallView.append(wallChecks, wallBody);
  move(wallView, 'wallMessage', 'yellowWallMessage');
  el('yellowWallMessage').textContent = '红黄墙共用此窗口；黄色墙需先启用视觉障碍检测。';
  const visionFeed = node('section', 'nav2-feed');
  visionFeed.append(node('h4', 'nav2-feed-title', '疑似障碍预览'));
  move(visionFeed, move(node('div', 'nav2-feed-controls'), check('visionPreviewEnabled', '显示疑似障碍预览')));
  visionFeed.append(imageBody(el('visionPreviewImage'), '勾选后查看近处疑似障碍，仅供观察'));
  move(visionFeed, 'visionPreviewMessage');
  feeds.append(wallView, visionFeed); media.append(feeds);
  const cameraSettings = details('导航预览参数', 'nav2MediaSettings');
  wallParams.prepend(node('h4', '', '虚拟墙投影'));
  cameraSettings.body.append(wallParams);
  const visionParams = move(node('div', 'nav2-fields'), label('visionHeight'), label('visionPitch'), label('visionScale'));
  cameraSettings.body.append(node('h4', '', '疑似障碍估距'), visionParams);
  media.append(cameraSettings.card);
  if (calibration) media.append(calibration); // 原校准闭包仍持有同一个完整容器。

  // 【操作分组】只调整导航面板已有的目标选择、人物规划与人工选点按钮。
  const workflows = node('div', 'nav2-workflows'); workflows.id = 'nav2Actions';
  const personCard = node('section', 'nav2-card'); personCard.id = 'nav2PersonActions';
  personCard.append(title('人物追踪', '选择 ID → 记录并规划 → 开始追踪'));
  el('navTargetForm').classList.add('nav2-fields');
  move(personCard, 'navTargetForm', 'navTargetState', 'navTargetMessage', 'previewRequestError', 'nav2PersonPanel');
  const personInfo = details('人物测量与规划详情');
  move(personInfo.body, 'previewTarget', 'previewStatus', targetHint);
  // 相同人物规划接口仅保留一个可见入口，保留原节点供既有状态脚本使用。
  el('previewPath').hidden = true; move(personInfo.body, 'previewPath'); personCard.append(personInfo.card);
  pointPanel.classList.add('nav2-card');
  pointPanel.prepend(title('地图选点行驶', '选取终点 → 预览路线 → 开始行驶'));
  pointPanel.append(move(node('div', 'nav2-actions'), 'pointPick', 'pointPreview', 'pointStart', 'pointStop'));
  move(pointPanel, 'pointMessage');
  el('pointStart').textContent = '开始行驶'; el('pointStart').title = '执行已预览路线并解除强停';
  el('pointStop').textContent = '停止行驶';
  workflows.append(personCard, pointPanel);

  // 【设置】折叠自有绘图、避障/缓冲与图例说明，不折叠错误和实时地图状态。
  const settings = node('div', 'nav2-settings'); settings.id = 'nav2Settings';
  const drawSettings = details('绘制地图禁区', 'nav2EditSettings');
  drawSettings.body.append(editor); editor.querySelector('.actions')?.classList.add('nav2-actions');
  const avoidSettings = details('视觉避障与紫色缓冲');
  move(avoidSettings.body, check('navVisionEnabled'), 'navVisionToggleMessage', 'visionLayerState',
    check('inflationEnabled'), label('inflationRadius'), 'inflationApply', 'inflationMessage');
  const mapDetails = details('地图图例与说明');
  move(mapDetails.body, legend, 'navObstacleMode', check('mapShowInflation'), inflationHint);
  mapDetails.body.append(node('p', 'hint', '橙色三角形为所选人物，紫点为其他人物，绿色方块为跟随停车终点；淡色人物为最后位置，灰色虚线为历史路线。'));
  settings.append(drawSettings.card, avoidSettings.card, mapDetails.card);

  // 【地图】原画布保持尺寸、投影、监听器和触摸规则；只改变面板内的摆放。
  const mapCard = node('section', 'nav2-map');
  // R28：仅把自有平移控件与缩放一起收纳，不改原页面结构。
  const toolbar = move(node('div', 'nav2-toolbar'), label('mapZoom'), 'mapPanControls');
  const instruction = node('p', 'hint', '查看时可上下、左右拖动地图；选点、绘制时使用对应操作。');
  mapCard.append(toolbar, instruction, mapView);
  move(mapCard, 'mapStatus', 'navLastStop');
  const overview = node('div', 'nav2-overview'); overview.append(mapCard, media);
  const notes = details('其他导航说明');
  for (const child of Array.from(panel.childNodes)) {
    if (child !== heading && child !== oldLayout) notes.body.append(child);
  }
  // 此时原布局仅余空白；不移除任何原控件或重新执行 script。
  for (const child of Array.from(oldLayout.childNodes)) notes.body.append(child);
  oldLayout.remove();
  mapDetails.body.append(notes.card);
  panel.append(heading, overview, workflows, settings);
  for (const input of panel.querySelectorAll('input[type="checkbox"]')) input.closest('label')?.classList.add('nav2-check');

  // 【兼容原门控】预览移出原隐藏父容器后，继续遵守地图模式和强停条件。
  let control = {enabled: !pointPanel.hidden, stopped: false};
  function availability() {
    const allowed = control.enabled && control.stopped && !pointPanel.hidden;
    el('visionPreviewEnabled').disabled = !allowed;
    for (const input of visionParams.querySelectorAll('input')) input.disabled = !allowed;
    drawSettings.card.hidden = editor.hidden;
  }
  window.addEventListener('nav2-map-control', event => { control = event.detail; availability(); });
  const gateObserver = new MutationObserver(availability);
  gateObserver.observe(pointPanel, {attributes: true, attributeFilter: ['hidden']});
  gateObserver.observe(editor, {attributes: true, attributeFilter: ['hidden']});
  observers.push(gateObserver); availability();
  el('pointPick').addEventListener('click', () => {
    instruction.textContent = '点击地图空地选取终点，然后点击预览路线。';
    mapCard.scrollIntoView({block: 'start', behavior: 'smooth'});
  });
  el('mapEditToggle').addEventListener('click', () => {
    instruction.textContent = el('obstacleMap').classList.contains('editing')
      ? '按住鼠标或手指绘画，完成后在“绘制地图禁区”中应用。' : '绘制已结束，可选择目标并预览路线。';
    if (el('obstacleMap').classList.contains('editing')) mapCard.scrollIntoView({block: 'start', behavior: 'smooth'});
  });
  window.addEventListener('beforeunload', () => observers.forEach(observer => observer.disconnect()), {once: true});
  panel.dataset.nav2Layout = 'ready';
})();
