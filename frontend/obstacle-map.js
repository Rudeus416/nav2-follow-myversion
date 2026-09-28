// 【内容标注】用途：导航小地图与选点/人物操作面板。
// 对应用户需求：R02 R03 R11 R15 R16 R17 R18 R27 R28 R29 R30 R31（原话及追溯边界见 nav2/CODE_GUIDE.md）。
// 添加/修改逻辑：绘制地图、路线和人物；编辑禁区；锁定历史点；启动/结束连续追踪，不直接发布速度。
// R27 仅整理自有导航面板；不移动原视频、强停或原页面其他区域。
(() => {
  const canvas = document.getElementById('obstacleMap');
  const status = document.getElementById('mapStatus');
  const zoom = document.getElementById('mapZoom');
  const showInflation = document.getElementById('mapShowInflation');
  const ctx = canvas.getContext('2d');
  let snapshot = null;
  let peopleMap=null,peopleReceived=0,personFollowing=false,peopleConnected=false;
  // 【职责 / R02 R03 R11 R15 R16 R17 R18】peopleUnavailable：接口中断时禁用启动，保留带过期提示的显示数据。
  function peopleUnavailable() {
    peopleConnected=false;personStart.disabled=true;
    if(peopleMap)peopleMap={...peopleMap,route:[],goal:null,people:peopleMap.people.map(p=>({...p,stale:true}))};
  }
  const personPanel=document.createElement('div');personPanel.id='nav2PersonPanel';
  const personStatus=document.createElement('p');personStatus.className='message';personStatus.setAttribute('role','status');
  const personStart=document.createElement('button');
  personStart.id='nav2PersonStart';personStart.textContent='开始追踪';personStart.type='button';personStart.disabled=true;
  const personPreview=document.createElement('button');
  personPreview.type='button';personPreview.id='nav2PersonPreview';personPreview.textContent='记录人物点并规划';
  let targetMarker=null,personStarting=false,personPlanning=false;
  // R30：操作失败与轮询状态分开，避免 0.5 秒后覆盖错误而看起来“按钮无反应”。
  const personActionError=document.createElement('p');personActionError.id='nav2PersonActionError';
  personActionError.className='error';personActionError.setAttribute('role','alert');personActionError.hidden=true;
  const personStartReason=document.createElement('p');personStartReason.id='nav2PersonStartReason';
  personStartReason.className='hint';personStartReason.setAttribute('role','status');
  function clearPersonError(){personActionError.textContent='';personActionError.hidden=true;}
  function showPersonError(action,error) {
    const reason=error.name==='TimeoutError' || error.name==='AbortError'
      ? (action==='开始追踪' ? '请求超时，执行状态尚未确认；请先强停并检查状态，再重试' : '请求超时，请检查连接和当前状态后重试') : error.message;
    personActionError.textContent=`上次${action}失败：${reason}`;personActionError.hidden=false;
  }
  function currentPersonTarget() {
    return peopleMap?.people?.find(p=>p.id===peopleMap.selected_id && !p.stale &&
      p.age+(performance.now()-peopleReceived)/1000<=1.2);
  }
  // R30：使用后端同一组前置检查；允许点击仍不替代执行端的整路线与车位复核。
  function personStartBlockedReason() {
    if(personStarting)return '正在复核并请求开始追踪，请稍候';
    if(personPlanning)return '正在请求人物路线规划，请稍候';
    if(!peopleConnected)return '人物接口未连接，等待启动条件更新';
    if(!control.stopped)return '请先强停，再规划或开始新的追踪';
    const readiness=peopleMap?.start_readiness?.[continuous.checked?'continuous':'single'];
    if(!readiness || typeof readiness.ready!=='boolean')return '启动条件尚未同步，请重启视觉服务加载更新后再试';
    if(!readiness.ready)return readiness.reason || '人物追踪启动条件未满足';
    // R31：实时三角形仅报告当前观测；已明确锁定的本段按预览与后端条件启动。
    const route=peopleMap?.person_preview;
    if(route?.state!=='ready' || !route.points?.length ||
       !(route.age+(performance.now()-peopleReceived)/1000<30))return '请先规划 30 秒内的有效人物路线';
    return '';
  }
  function updatePersonButtons() {
    const reason=personStartBlockedReason();
    personStart.disabled=!!reason;
    personStart.textContent=personStarting?'正在启动…':'开始追踪';
    personStart.title=reason || '点击后复核并执行预览路线，强停可随时停止';
    personStartReason.textContent=reason || '可开始追踪；复核通过后驶向已锁定人物点，本段不随人物闪烁或移动改线。';
    personPreview.disabled=personPlanning || personStarting || !control.stopped;
    personPreview.textContent=personPlanning?'正在请求规划…':'记录人物点并规划';
    continuous.disabled=personStarting || personPlanning || !control.stopped;
  }
  // 【职责 / R02 R03 R11 R15 R16 R17 R18】previewPerson：点击锁定最近人物世界坐标并请求停车规划。
  async function previewPerson(event) {
    if(event){event.preventDefault();event.stopImmediatePropagation();}
    if(personPlanning || personStarting)return;
    clearPersonError();
    if(!control.stopped){showPersonError('人物规划',new Error('请先强停，再记录人物点并规划'));return;}
    personPlanning=true;updatePersonButtons();
    try {
      const r=await fetch('/api/nav2/person-preview',{method:'POST',signal:AbortSignal.timeout(5000)});
      const data=await r.json();if(!r.ok)throw new Error(data.detail || '人物规划失败');
      targetMarker=data.target_marker;
      window.dispatchEvent(new CustomEvent('nav2-preview',{detail:data.preview}));
      personStatus.textContent=data.preview.message;peopleConnected=false;draw();
    } catch(e){showPersonError('人物规划',e);}
    finally {personPlanning=false;updatePersonButtons();}
  }
  personPreview.addEventListener('click',previewPerson);
  // Navigation-only capture handler replaces the old live-only preview request.
  // Original app.js and its non-navigation controls are left untouched.
  document.getElementById('previewPath')?.addEventListener('click',previewPerson,true);
  const continuous=document.createElement('input');continuous.type='checkbox';continuous.checked=true;continuous.style.width='auto';
  continuous.addEventListener('change',updatePersonButtons);
  const continuousLabel=document.createElement('label');continuousLabel.append(continuous,document.createTextNode(' 连续追踪：到达后更新终点，1 米内等待'));
  const personEnd=document.createElement('button');personEnd.id='nav2PersonEnd';personEnd.className='secondary';personEnd.type='button';personEnd.textContent='结束追踪';
  personEnd.addEventListener('click',async()=>{
    clearPersonError();
    try {const r=await fetch('/api/nav2/person-end',{method:'POST'});const data=await r.json();if(!r.ok)throw new Error(data.detail || '停止失败');startHint.textContent=data.message;}
    catch(e){showPersonError('结束追踪',e);}
  });
  const personActions=document.createElement('div');personActions.className='nav2-actions';
  personActions.style.cssText='display:flex;flex-wrap:wrap;gap:10px;align-items:center';
  personPreview.style.width=personStart.style.width='auto';
  personActions.append(personPreview,personStart,personEnd);
  const startHint=document.createElement('p');startHint.className='hint';
  startHint.textContent='规划完成后点击开始追踪，沿已锁定路线行驶；连续模式到达后再定位人物，单次模式到达即结束。强停可随时停止。';
  personPanel.append(personStatus,continuousLabel,personActions,personActionError,personStartReason,startHint);status.after(personPanel);
  personStart.addEventListener('click',async()=>{
    if(personStarting || personPlanning)return;
    clearPersonError();
    const reason=personStartBlockedReason();
    if(reason){showPersonError('开始追踪',new Error(reason));updatePersonButtons();return;}
    personStarting=true;updatePersonButtons();
    try {
      const r=await fetch('/api/nav2/person-execute',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({sequence:peopleMap?.preview_sequence,continuous:continuous.checked}),signal:AbortSignal.timeout(10000)});
      const data=await r.json();if(!r.ok)throw new Error(data.detail || '启动失败');
      if(data.target_marker)targetMarker=data.target_marker;
      if(data.preview)window.dispatchEvent(new CustomEvent('nav2-preview',{detail:data.preview}));
      personStatus.textContent=data.message;startHint.textContent=data.message;draw();
    } catch(e) {showPersonError('开始追踪',e);}
    finally {personStarting=false;updatePersonButtons();}
  });
  const el=id=>document.getElementById(id);
  let control={enabled:false,stopped:false}, editing=false, busy=false;
  let draft=null, drag=null, projection=null, draftVersion=null;
  let picking=false, selectedGoal=null;
  // 【视图 / R28 R29】双轴平移只改变网页投影，单位为地图米；不改变终点或发送控制请求。
  let panX=0, panY=0, panDrag=null, viewKey=null;
  const panControls=document.createElement('div');panControls.id='mapPanControls';
  // 共用控件构建和状态更新，保证上下/左右的禁用、范围、重置规则一致。
  function createPanControl(axis,title,negative,positive) {
    const group=document.createElement('div');group.className='nav2-pan-control';
    const caption=document.createElement('div');caption.className='nav2-pan-caption';
    const label=document.createElement('label');label.htmlFor=axis==='x'?'mapPanX':'mapPanY';label.textContent=title;
    const value=document.createElement('output');value.id=axis==='x'?'mapPanXValue':'mapPanValue';value.htmlFor=label.htmlFor;
    const row=document.createElement('div');row.className='nav2-pan-row';
    const range=document.createElement('input');range.id=label.htmlFor;range.type='range';
    range.min='0';range.max='0';range.step='0.01';range.value='0';range.disabled=true;
    const buttons=[negative,positive].map(([id,text,name])=>{
      const button=document.createElement('button');button.id=id;button.type='button';button.className='secondary';
      button.textContent=text;button.title=name;button.setAttribute('aria-label',name);button.disabled=true;return button;
    });
    caption.append(label,value);row.append(buttons[0],range,buttons[1]);group.append(caption,row);panControls.append(group);
    const move=value=>{endPan();if(axis==='x')setPan(value,panY);else setPan(panX,value);};
    range.addEventListener('input',()=>move(Number(range.value)));
    buttons[0].addEventListener('click',()=>move((axis==='x'?panX:panY)-0.25));
    buttons[1].addEventListener('click',()=>move((axis==='x'?panX:panY)+0.25));
    return {axis,range,value,buttons,negative:negative[2],positive:positive[2]};
  }
  const panAxes=[
    createPanControl('y','地图上下移动',['mapPanUp','↑','地图上移'],['mapPanDown','↓','地图下移']),
    createPanControl('x','地图左右移动',['mapPanLeft','←','地图左移'],['mapPanRight','→','地图右移'])
  ];
  zoom.closest('label').after(panControls);
  function endPan() {
    if(!panDrag)return;
    const id=panDrag.id;panDrag=null;
    if(canvas.hasPointerCapture(id))canvas.releasePointerCapture(id);
  }
  function syncPanControls() {
    for(const {axis,range,value,buttons,negative,positive} of panAxes) {
      const offset=axis==='x'?panX:panY,limit=projection ? (axis==='x'?projection.wx:projection.wy)/2 : 0;
      range.min=String(-limit);range.max=String(limit);range.value=String(offset);
      const label=Math.abs(offset)<0.005 ? '居中' : `${offset<0?negative.slice(2):positive.slice(2)} ${Math.abs(offset).toFixed(2)} 米`;
      value.textContent=label;range.setAttribute('aria-valuetext',label);
      range.disabled=!projection || !!drag || busy;
      buttons[0].disabled=range.disabled || offset<=-limit;
      buttons[1].disabled=range.disabled || offset>=limit;
    }
    const canPan=!!projection && !editing && !picking && !busy;
    if(!canPan)endPan();
    canvas.classList.toggle('pan-ready',canPan);
    canvas.classList.toggle('panning',!!panDrag);
  }
  function setPan(x,y) {
    if(!projection || !Number.isFinite(x) || !Number.isFinite(y))return;
    panX=Math.max(-projection.wx/2,Math.min(projection.wx/2,x));
    panY=Math.max(-projection.wy/2,Math.min(projection.wy/2,y));draw();
  }
  // 【职责 / R02 R03 R11 R15 R16 R17 R18】editorButtons：按强停、地图版本和操作状态更新按钮可用性。
  function editorButtons() {
    el('mapEditor').hidden=!control.enabled;
    el('pointPreviewPanel').hidden=!control.enabled;
    el('pointPick').disabled=!control.enabled || !control.stopped || !snapshot?.editing?.ready;
    el('pointPreview').disabled=!control.stopped || !selectedGoal;
    el('pointStart').disabled=!control.enabled || !control.stopped || !selectedGoal || preview?.state!=='ready' || (preview.age || 0)+(performance.now()-previewReceived)/1000>=30;
    const allowed=control.enabled && control.stopped && snapshot?.editing?.ready && !busy;
    if(!allowed) {editing=false;drag=null;draft=null;draftVersion=null;}
    canvas.classList.toggle('editing',editing);
    syncPanControls();
    el('mapEditToggle').disabled=!allowed;
    el('mapEditToggle').textContent=editing?'结束绘制':'绘制阻挡区';
    el('mapEditApply').disabled=!allowed || !draft;
    el('mapEditDiscard').disabled=!draft;
    const count=snapshot?.editing?.rectangles?.length || 0;
    el('mapEditUndo').disabled=!allowed || !count;
    el('mapEditClear').disabled=!allowed || !count;
    if(control.enabled && !control.stopped) el('mapEditMessage').textContent='请先强制停止，再编辑地图。';
  }
  window.addEventListener('nav2-map-control',event=>{control=event.detail;editorButtons();updatePersonButtons();draw();});
  // 【职责 / R02 R03 R11 R15 R16 R17 R18】paint：将鼠标/触摸笔画离散成禁区栅格。
  function paint(a,b) {
    const map=snapshot.layers.static_map, r=(Number(el('mapBrushWidth').value)-1)/2;
    const steps=Math.max(Math.abs(b[0]-a[0]),Math.abs(b[1]-a[1]),1);
    const cells=new Set(draft.cells.map(p=>p.join(',')));
    for(let i=0;i<=steps;i++) {
      const x=Math.round(a[0]+(b[0]-a[0])*i/steps), y=Math.round(a[1]+(b[1]-a[1])*i/steps);
      for(let dx=-r;dx<=r;dx++)for(let dy=-r;dy<=r;dy++) {
        if(x+dx>=0 && x+dx<map.width && y+dy>=0 && y+dy<map.height && cells.size<20000)cells.add(`${x+dx},${y+dy}`);
      }
    }
    draft.cells=Array.from(cells,key=>key.split(',').map(Number));
  }
  for(const id of ['mapDrawTool','mapBrushWidth'])el(id).addEventListener('change',()=>{draft=null;drag=null;editorButtons();draw();});
  el('pointPick').addEventListener('click',()=>{picking=true;editing=false;drag=null;draft=null;editorButtons();el('pointMessage').textContent='请点击地图空地选择终点，再预览路线。';});
  el('pointPreview').addEventListener('click',async()=>{
    if(!selectedGoal || !control.stopped)return;
    try {
      const response=await fetch('/api/nav2/point-preview',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(selectedGoal)});
      const value=await response.json();if(!response.ok)throw new Error(value.detail);
      window.dispatchEvent(new CustomEvent('nav2-preview',{detail:value}));el('pointMessage').textContent=value.message+'（仅规划，不执行）';
    }catch(error){el('pointMessage').textContent=error.message;}
  });
  el('pointStart').addEventListener('click',async()=>{
    el('pointStart').disabled=true;
    try{
      const r=await fetch('/api/nav2/point-start',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(selectedGoal)});
      const value=await r.json();if(!r.ok)throw new Error(value.detail || '启动失败');
      el('pointMessage').textContent=value.message;
    }catch(e){el('pointMessage').textContent=e.message;}
  });
  el('pointStop').addEventListener('click',async()=>{
    try{
      const r=await fetch('/api/nav2/point-stop',{method:'POST'});
      if(!r.ok)throw new Error('停止请求失败，请使用强停');
      el('pointMessage').textContent='已强停并取消终点导航';
    }catch(e){el('pointMessage').textContent=e.message;}
  });
  // 【职责 / R02 R03 R11 R15 R16 R17 R18】cell：把屏幕坐标换成地图栅格索引。
  function cell(event,clamp=false) {
    const map=snapshot?.layers.static_map;
    if(!map || !projection)return null;
    const bounds=canvas.getBoundingClientRect();
    if(!bounds.width || !bounds.height)return null;
    const x=(event.clientX-bounds.left)*canvas.width/bounds.width;
    const y=(event.clientY-bounds.top)*canvas.height/bounds.height;
    // R28/R29：反算使用与绘图相同的平移中心；图外空白不再被夹取成边缘终点/禁区。
    const gx=Math.floor(((x-projection.centerX)/projection.scale+projection.wx/2)/map.resolution);
    const gy=Math.floor(((projection.centerY-y)/projection.scale+projection.wy/2)/map.resolution);
    // 已开始的笔画拖出地图时，沿用原有的边缘夹取；空白处不能开始选点或画新笔画。
    if(clamp)return [Math.max(0,Math.min(map.width-1,gx)),Math.max(0,Math.min(map.height-1,gy))];
    if(x<0 || x>=canvas.width || y<0 || y>=canvas.height || gx<0 || gx>=map.width || gy<0 || gy>=map.height)return null;
    return [gx,gy];
  }
  canvas.addEventListener('pointerdown',event=>{
    if(!event.isPrimary || event.button!==0 || panDrag || drag)return;
    if(picking && control.stopped && snapshot?.editing?.ready) {
      const p=cell(event);if(!p)return;event.preventDefault();picking=false;
      selectedGoal={x:p[0],y:p[1],base_id:snapshot.editing.base_id,revision:snapshot.editing.revision};
      el('pointMessage').textContent=`已选栅格 (${p[0]}, ${p[1]})，请点击预览；不会驱动车辆。`;editorButtons();draw();return;
    }
    // R28/R29：选点/画禁区优先；仅查看状态可拖动，单独记录指针，避免把拖图当作绘制。
    if(!editing && !picking && !busy && projection) {
      event.preventDefault();panDrag={id:event.pointerId,lastX:event.clientX,lastY:event.clientY};
      canvas.setPointerCapture(event.pointerId);syncPanControls();return;
    }
    if(!editing || busy || !control.stopped || !snapshot?.editing?.ready || snapshot.layers.static_map?.stale)return;
    const point=cell(event);if(!point)return;
    event.preventDefault();canvas.setPointerCapture(event.pointerId);
    drag={point,id:event.pointerId};
    if(el('mapDrawTool').value==='brush') {draft={cells:[]};paint(point,point);}
    else draft=[point[0],point[1],point[0]+1,point[1]+1];
    draftVersion={base_id:snapshot.editing.base_id,revision:snapshot.editing.revision};
    editorButtons();draw();
  });
  canvas.addEventListener('pointermove',event=>{
    if(panDrag?.id===event.pointerId && projection) {
      event.preventDefault();
      const bounds=canvas.getBoundingClientRect();
      if(bounds.width>0 && bounds.height>0) {
        const dx=(event.clientX-panDrag.lastX)*canvas.width/bounds.width/projection.scale;
        const dy=(event.clientY-panDrag.lastY)*canvas.height/bounds.height/projection.scale;
        panDrag.lastX=event.clientX;panDrag.lastY=event.clientY;setPan(panX+dx,panY+dy);
      }
      return;
    }
    if(!drag || drag.id!==event.pointerId)return;
    event.preventDefault();const p=cell(event,true);if(!p)return;
    const a=drag.point;
    if(draft.cells) {paint(a,p);drag.point=p;}
    else draft=[Math.min(a[0],p[0]),Math.min(a[1],p[1]),Math.max(a[0],p[0])+1,Math.max(a[1],p[1])+1];
    draw();
  });
  // R28：只结束对应指针的事务；额外手指与失去捕获不会误清除另一笔画。
  function finishPointer(event,cancelled) {
    if(panDrag?.id===event.pointerId) {endPan();syncPanControls();return;}
    if(drag?.id!==event.pointerId)return;
    drag=null;
    if(canvas.hasPointerCapture(event.pointerId))canvas.releasePointerCapture(event.pointerId);
    if(cancelled) {draft=null;draftVersion=null;}
    editorButtons();draw();
  }
  canvas.addEventListener('pointerup',event=>finishPointer(event,false));
  canvas.addEventListener('pointercancel',event=>finishPointer(event,true));
  canvas.addEventListener('lostpointercapture',event=>finishPointer(event,true));
  el('mapEditToggle').addEventListener('click',()=>{picking=false;editing=!editing;drag=null;editorButtons();draw();});
  el('mapEditDiscard').addEventListener('click',()=>{draft=null;drag=null;editorButtons();draw();});
  // 【职责 / R02 R03 R11 R15 R16 R17 R18】applyEdit：提交编辑动作并使旧预览失效。
  async function applyEdit(action) {
    if(busy || !control.stopped || !snapshot?.editing?.ready)return;
    const version=action==='add'?draftVersion:snapshot.editing;
    const rect=draft;
    if(action==='add' && !rect)return;
    busy=true;editorButtons();
    try {
      const response=await fetch('/api/nav2/map-edit',{method:'POST',headers:{'Content-Type':'application/json'},
        body:JSON.stringify({action,rect,base_id:version.base_id,revision:version.revision})});
      const result=await response.json();if(!response.ok)throw new Error(result.detail || '编辑失败');
      snapshot.editing=result;draft=null;preview=null;
      window.dispatchEvent(new CustomEvent('nav2-map-snapshot',{detail:snapshot}));
      el('mapEditMessage').textContent=`已保存并发布，当前 ${result.rectangles.length} 个新增阻挡区。等待地图更新后重新预览。`;
      const fresh=await fetch('/api/obstacle-map',{cache:'no-store'});if(fresh.ok)snapshot=await fresh.json();
    } catch(error) {el('mapEditMessage').textContent=error.message;}
    finally {busy=false;editorButtons();draw();}
  }
  el('mapEditApply').addEventListener('click',()=>applyEdit('add'));
  el('mapEditUndo').addEventListener('click',()=>applyEdit('undo'));
  el('mapEditClear').addEventListener('click',()=>applyEdit('clear'));

  let preview = null, previewReceived = 0;
  window.addEventListener('nav2-preview', event => {
    preview = event.detail; previewReceived = performance.now();
    const notice=el('mapPathValidity');
    const invalid=preview?.state==='error' || (preview?.state==='ready' && !preview.points?.length);
    notice.hidden=!invalid;
    notice.textContent=invalid ? '无有效路径'+(preview?.message ? '：'+preview.message : '') : '';
    draw();
  });
  // 【职责 / R02 R03 R11 R15 R16 R17 R18】draw：按地图变换绘制图层、人物、路线及明确区分的历史点。
  function draw() {
    projection=null;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    const layers = snapshot?.layers || {};
    const map = layers.static_map || layers.map;
    if (!map || map.stale) {
      endPan();syncPanControls();
      ctx.fillStyle = '#cbd5e1'; ctx.font = '20px sans-serif';
      ctx.fillText(map ? '地图已过期，等待更新' : '等待静态地图或 Nav2 地图', 25, 50);
      return;
    }
    const c = Math.cos(map.yaw), s = Math.sin(map.yaw);
    const wx = map.width * map.resolution, wy = map.height * map.resolution;
    const center = [map.origin[0] + c*wx/2-s*wy/2, map.origin[1]+s*wx/2+c*wy/2];
    const scale = Math.min(canvas.width / wx, canvas.height / wy) * 0.9 * Number(zoom.value);
    // R28/R29：普通轮询/缩放保留平移；换底图坐标基准或尺寸才复位，避免旧手势跳到新图。
    const key=JSON.stringify([!!layers.static_map,map.width,map.height,map.resolution,map.yaw,
      ...(layers.static_map ? map.origin : [])]);
    if(viewKey!==null && viewKey!==key) {endPan();panX=0;panY=0;}
    viewKey=key;
    panX=Math.max(-wx/2,Math.min(wx/2,panX));panY=Math.max(-wy/2,Math.min(wy/2,panY));
    const centerX=canvas.width/2+panX*scale,centerY=canvas.height/2+panY*scale;
    projection={scale,wx,wy,centerX,centerY};syncPanControls();
    // Orient the view with the selected map, so the corridor points upward.
    const screen = ([x,y]) => {
      const dx=x-center[0], dy=y-center[1];
      return [centerX+(c*dx+s*dy)*scale, centerY-(-s*dx+c*dy)*scale];
    };
    const tiles = document.createElement('canvas'); tiles.width=map.width; tiles.height=map.height;
    const tc = tiles.getContext('2d'); const pixels=tc.createImageData(map.width,map.height);
    map.data.forEach((v,i) => {
      const color = v < 0 ? [88,96,109] : v >= 99 ? (layers.static_map ? [35,42,52] : [240,65,75]) : v === 0 || !showInflation.checked ? [225,232,240] : [150,95,190];
      pixels.data.set([...color,255],i*4);
    });
    tc.putImageData(pixels,0,0);
    const origin=screen(map.origin);
    ctx.save(); ctx.translate(...origin); ctx.rotate(0); ctx.scale(scale*map.resolution,-scale*map.resolution);
    ctx.imageSmoothingEnabled=false; ctx.drawImage(tiles,0,0); ctx.restore();
    const costs=layers.static_map && layers.map;
    if(costs && !costs.stale) {
      const overlay=document.createElement('canvas'); overlay.width=costs.width; overlay.height=costs.height;
      const oc=overlay.getContext('2d'), pixels=oc.createImageData(costs.width,costs.height);
      costs.data.forEach((v,i)=>{if(v>=99 || (v>0 && showInflation.checked))pixels.data.set(v>=99?[240,65,75,190]:[150,95,190,100],i*4);});
      oc.putImageData(pixels,0,0);
      ctx.save();
      ctx.beginPath();ctx.rect(centerX-wx*scale/2,centerY-wy*scale/2,wx*scale,wy*scale);ctx.clip();
      ctx.translate(...screen(costs.origin));ctx.rotate(-(costs.yaw-map.yaw));
      ctx.scale(scale*costs.resolution,-scale*costs.resolution);ctx.imageSmoothingEnabled=false;ctx.drawImage(overlay,0,0);ctx.restore();
    }
    if(layers.static_map) {
      const rectScreen=([x0,y0,x1,y1])=>[centerX+(x0*map.resolution-wx/2)*scale,
        centerY-(y1*map.resolution-wy/2)*scale,(x1-x0)*map.resolution*scale,(y1-y0)*map.resolution*scale];
      const drawShape=shape=>{
        if(shape.cells) {for(const [x,y] of shape.cells)ctx.fillRect(...rectScreen([x,y,x+1,y+1]));}
        else ctx.fillRect(...rectScreen(shape));
      };
      if(selectedGoal && selectedGoal.base_id===snapshot.editing?.base_id) {ctx.save();ctx.strokeStyle='#e11d48';ctx.lineWidth=4;ctx.strokeRect(...rectScreen([selectedGoal.x,selectedGoal.y,selectedGoal.x+1,selectedGoal.y+1]));ctx.restore();}
      ctx.fillStyle='#111111';
      for(const rect of snapshot.editing?.rectangles || [])drawShape(rect);
      if(draft) {ctx.save();ctx.fillStyle='rgba(0,0,0,0.4)';ctx.strokeStyle='#06b6d4';ctx.lineWidth=3;ctx.setLineDash([8,5]);drawShape(draft);if(!draft.cells)ctx.strokeRect(...rectScreen(draft));ctx.restore();}
    }
    for (const [name,color] of [['path','#15803d'],['footprint','#0077ff'],['radar','#f59e0b']]) {
      // Planner publishes /plan before our footprint validation completes.
      // While stopped show only the validated preview, never that raw candidate.
      if(name==='path' && (control.stopped || personFollowing)) continue;
      const layer=layers[name]; if (!layer || layer.stale || !layer.points.length) continue;
      ctx.strokeStyle=color; ctx.fillStyle=color; ctx.lineWidth=3;
      if (name==='radar') {
        for (const point of layer.points) {const [x,y]=screen(point);ctx.beginPath();ctx.arc(x,y,2.5,0,2*Math.PI);ctx.fill();}
      } else {
        ctx.beginPath(); layer.points.forEach((p,i)=>{const [x,y]=screen(p);if(i)ctx.lineTo(x,y);else ctx.moveTo(x,y);});
        if(name==='footprint')ctx.closePath(); ctx.stroke();
      }
    }
    if ((!personFollowing || peopleMap?.person_preview) && preview?.state === 'ready' && (preview.age || 0) + (performance.now()-previewReceived)/1000 < 30) {
      ctx.save(); ctx.strokeStyle='#06b6d4'; ctx.lineWidth=4; ctx.setLineDash([10,6]); ctx.beginPath();
      preview.points.forEach((p,i)=>{const [x,y]=screen(p);if(i)ctx.lineTo(x,y);else ctx.moveTo(x,y);});
      ctx.stroke();ctx.restore();
    }
    if(peopleMap && (performance.now()-peopleReceived)<3000) {
      const elapsed=(performance.now()-peopleReceived)/1000;
      if(peopleConnected && elapsed<2 && peopleMap.following && !control.stopped && peopleMap.route?.length) {
        ctx.save();ctx.strokeStyle='#15803d';ctx.lineWidth=4;ctx.beginPath();
        peopleMap.route.forEach((p,i)=>{const [x,y]=screen(p);if(i)ctx.lineTo(x,y);else ctx.moveTo(x,y);});
        ctx.stroke();ctx.restore();
        if(peopleMap.goal) {const [x,y]=screen(peopleMap.goal);ctx.fillStyle='#15803d';ctx.fillRect(x-5,y-5,10,10);ctx.fillText('跟随停车终点',x+9,y+20);}
      }
      if(peopleMap.historical_route?.length && peopleMap.history_age+elapsed<=15) {
        // History is display-only: never pass it to preview or start controls.
        ctx.save();ctx.strokeStyle='#94a3b8';ctx.lineWidth=3;ctx.setLineDash([5,7]);ctx.beginPath();
        peopleMap.historical_route.forEach((p,i)=>{const [x,y]=screen(p);if(i)ctx.lineTo(x,y);else ctx.moveTo(x,y);});
        ctx.stroke();ctx.setLineDash([]);ctx.fillStyle='#64748b';ctx.font='18px sans-serif';
        const [hx,hy]=screen(peopleMap.historical_route.at(-1));
        ctx.fillText('历史路线（已停止，不可据此行驶）',hx+10,hy+24);ctx.restore();
      }
      for(const person of peopleMap.people || []) {
        if(person.age+elapsed>3)continue;
        const stale=person.stale || person.age+elapsed>1.2 || !peopleConnected;
        const [x,y]=screen(person.position),selected=person.id===peopleMap.selected_id;
        ctx.save();ctx.fillStyle=selected?'#f97316':'#a855f7';ctx.strokeStyle='white';ctx.lineWidth=2;
        if(stale){ctx.globalAlpha=.5;ctx.setLineDash([3,3]);}
        ctx.beginPath();
        if(selected) {
          // Fixed screen-up triangle marks selection, not the person's heading.
          ctx.moveTo(x,y-12);ctx.lineTo(x+10,y+6);ctx.lineTo(x-10,y+6);ctx.closePath();
        } else {ctx.arc(x,y,6,0,2*Math.PI);}
        ctx.fill();ctx.stroke();
        ctx.font='18px sans-serif';ctx.fillText(`人物 ID ${person.id}${selected?'（目标）':''}${stale?' · 最后位置，待确认':''}`,x+12,y-12);ctx.restore();
      }
    }
    if(targetMarker) {
      const [x,y]=screen(targetMarker.position);
      ctx.save();ctx.strokeStyle='#c2410c';ctx.fillStyle='#ffedd5';ctx.lineWidth=3;
      ctx.beginPath();ctx.arc(x,y,10,0,Math.PI*2);ctx.fill();ctx.stroke();
      ctx.fillStyle='#c2410c';ctx.font='18px sans-serif';
      ctx.fillText(`历史人物点 ID ${targetMarker.id}（点击锁定）`,x+14,y+20);ctx.restore();
    }
    const robot=layers.robot;
    if(robot && !robot.stale) {
      const [x,y]=screen(robot.position), a=robot.yaw-map.yaw;
      ctx.save(); ctx.translate(x,y); ctx.rotate(-a);
      ctx.fillStyle='#0077ff'; ctx.strokeStyle='white'; ctx.lineWidth=2;
      ctx.beginPath(); ctx.moveTo(17,0); ctx.lineTo(-11,-10); ctx.lineTo(-6,0); ctx.lineTo(-11,10); ctx.closePath(); ctx.fill(); ctx.stroke(); ctx.restore();
      ctx.fillStyle='#005bbb'; ctx.font='20px sans-serif'; ctx.fillText('小车当前位置',x+20,y-15);
    }
    ctx.fillStyle='#111827'; ctx.fillRect(18,canvas.height-30,scale,4);
    ctx.font='16px sans-serif'; ctx.fillText('1 m',18,canvas.height-37);
  }
  // 【职责 / R02 R03 R11 R15 R16 R17 R18】refresh：异步刷新本面板数据，处理过期响应及资源清理。
  async function refresh() {
    try {
      if (!document.hidden) {
        const [response,peopleResponse]=await Promise.all([
          fetch('/api/obstacle-map',{cache:'no-store',signal:AbortSignal.timeout(3000)}),
          fetch('/api/nav2/people-map',{cache:'no-store',signal:AbortSignal.timeout(2000)}).catch(()=>null)
        ]);
        peopleUnavailable();
        if(peopleResponse?.ok) {
          try {peopleMap=await peopleResponse.json();peopleReceived=performance.now();personFollowing=peopleMap.following;peopleConnected=true;
            targetMarker=peopleMap.target_marker || null;
            if(peopleMap.person_preview)window.dispatchEvent(new CustomEvent('nav2-preview',{detail:peopleMap.person_preview}));
          } catch(_) {}
        }
        const target=currentPersonTarget();
        updatePersonButtons();
        personStatus.textContent=peopleMap ? `目标 ID ${peopleMap.selected_id}：${peopleConnected && target?'位置有效':(targetMarker?'当前人物暂不可见或数据过期；已点击锁定的历史点仍保留，路线状态见下方':'当前人物暂不可见或数据过期；请先记录有效人物点并规划')}。${peopleConnected?((peopleMap.person_preview?.message || '')+' '+(peopleMap.message || '')):'人物接口暂不可用'}${peopleMap.historical_route?.length?'；灰色虚线为历史路线，已停止执行。'+(peopleMap.stop_reason || ''):''}${control.stopped?'；当前强停，标点不会驱动车辆':''}` : '人物定位数据暂不可用';
        if(!response.ok)throw new Error(`HTTP ${response.status}`);
        snapshot=await response.json();
        window.dispatchEvent(new CustomEvent('nav2-map-snapshot',{detail:snapshot}));
        if(draftVersion && (draftVersion.base_id!==snapshot.editing?.base_id || draftVersion.revision!==snapshot.editing?.revision)) {draft=null;drag=null;draftVersion=null;}
        editorButtons();draw();
        const labels={...(snapshot.layers.static_map ? {static_map:'静态地图'} : {map:'障碍地图'}),robot:'小车位置',radar:'雷达',path:'导航路线'};
        status.textContent=Object.entries(labels).map(([key,label])=>{
          const layer=snapshot.layers[key];return `${label}：${layer ? `${layer.age.toFixed(1)} 秒前${layer.stale?'（过期）':''}`:'等待数据'}`;
        }).join(' · ') + (Object.keys(snapshot.errors).length ? ' | '+Object.entries(snapshot.errors).map(([k,v])=>`${labels[k]||k}：${v}`).join('；') : '');
      }
    } catch (error) {
      // Keep only a labelled static background; stale motion/obstacle data is hidden.
      const cached=snapshot?.layers?.static_map;
      snapshot=cached ? {layers:{static_map:cached},editing:{ready:false},errors:{}} : null;
      peopleUnavailable();updatePersonButtons();personStatus.textContent='地图连接中断；保留短时最后位置，仅供查看';
      preview=null;draft=null;drag=null;picking=false;selectedGoal=null;
      window.dispatchEvent(new CustomEvent('nav2-map-snapshot',{detail:null}));
      editorButtons();draw();
      if(cached){ctx.fillStyle='#fbbf24';ctx.font='20px sans-serif';ctx.fillText('缓存底图 · 连接中断，实时图层已隐藏',18,28);}
      status.textContent=`地图连接失败：${error.message}；${cached?'仅显示缓存底图，地图操作已禁用':'等待连接恢复'}`;
    } finally {setTimeout(refresh,500);}
  }
  zoom.addEventListener('input',()=>{endPan();draw();});
  showInflation.addEventListener('change',draw);
  document.getElementById('mapReset').addEventListener('click',()=>{endPan();panX=0;panY=0;zoom.value='1';draw();});
  document.addEventListener('visibilitychange',()=>{if(document.hidden){endPan();snapshot=null;draw();}});
  refresh();
})();


// 【布局接入 / R27】只加载自有导航面板样式和布局，不改原模板与页面其他区域。
(() => {
  function loadPanel() {
    if (document.getElementById('nav2PanelScript')) return;
    const panel = document.querySelector('.obstacle-map-panel');
    if (!panel) return;
    const notice = document.createElement('p'); notice.className = 'message';
    notice.setAttribute('role', 'status'); notice.textContent = '正在整理导航面板…'; panel.append(notice);
    const fail = message => { notice.textContent = message + '；请强制刷新页面后重试。'; panel.append(notice); };
    const script = document.createElement('script'); script.id = 'nav2PanelScript';
    script.src = '/assets/nav2-panel.js?v=20260928-5';
    script.onerror = () => fail('导航面板脚本加载失败');
    script.onload = () => {
      if (panel.dataset.nav2Layout === 'ready') notice.remove();
      else fail('导航面板整理未完成');
    };
    const marker = document.createElement('meta'); marker.id = script.id; document.head.append(marker);
    const css = document.createElement('link'); css.rel = 'stylesheet'; css.href = '/assets/nav2-panel.css?v=20260928-5';
    css.onerror = () => fail('导航面板样式加载失败');
    css.onload = () => marker.replaceWith(script);
    document.head.append(css);
  }
  if (document.readyState === 'complete') loadPanel();
  else {
    document.addEventListener('DOMContentLoaded', loadPanel, {once: true});
    window.addEventListener('load', loadPanel, {once: true});
  }
})();
