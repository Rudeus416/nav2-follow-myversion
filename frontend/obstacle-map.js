(() => {
  const canvas = document.getElementById('obstacleMap');
  const status = document.getElementById('mapStatus');
  const zoom = document.getElementById('mapZoom');
  const showInflation = document.getElementById('mapShowInflation');
  const ctx = canvas.getContext('2d');
  let snapshot = null;
  const el=id=>document.getElementById(id);
  let control={enabled:false,stopped:false}, editing=false, busy=false;
  let draft=null, drag=null, projection=null, draftVersion=null;
  let picking=false, selectedGoal=null;
  function editorButtons() {
    el('mapEditor').hidden=!control.enabled;
    el('pointPreviewPanel').hidden=!control.enabled;
    el('pointPick').disabled=!control.enabled || !control.stopped || !snapshot?.editing?.ready;
    el('pointPreview').disabled=!control.stopped || !selectedGoal;
    el('pointStart').disabled=!control.enabled || !control.stopped || !selectedGoal || preview?.state!=='ready' || (preview.age || 0)+(performance.now()-previewReceived)/1000>=30;
    const allowed=control.enabled && control.stopped && snapshot?.editing?.ready && !busy;
    if(!allowed) {editing=false;drag=null;draft=null;draftVersion=null;}
    canvas.classList.toggle('editing',editing);
    el('mapEditToggle').disabled=!allowed;
    el('mapEditToggle').textContent=editing?'结束绘制':'绘制阻挡区';
    el('mapEditApply').disabled=!allowed || !draft;
    el('mapEditDiscard').disabled=!draft;
    const count=snapshot?.editing?.rectangles?.length || 0;
    el('mapEditUndo').disabled=!allowed || !count;
    el('mapEditClear').disabled=!allowed || !count;
    if(control.enabled && !control.stopped) el('mapEditMessage').textContent='请先强制停止，再编辑地图。';
  }
  window.addEventListener('nav2-map-control',event=>{control=event.detail;editorButtons();draw();});
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
  function cell(event) {
    const map=snapshot?.layers.static_map;
    if(!map || !projection)return null;
    const bounds=canvas.getBoundingClientRect();
    const x=(event.clientX-bounds.left)*canvas.width/bounds.width;
    const y=(event.clientY-bounds.top)*canvas.height/bounds.height;
    return [Math.max(0,Math.min(map.width-1,Math.floor(((x-canvas.width/2)/projection.scale+projection.wx/2)/map.resolution))),
            Math.max(0,Math.min(map.height-1,Math.floor(((canvas.height/2-y)/projection.scale+projection.wy/2)/map.resolution)))];
  }
  canvas.addEventListener('pointerdown',event=>{
    if(picking && control.stopped && snapshot?.editing?.ready) {
      const p=cell(event);if(!p)return;event.preventDefault();picking=false;
      selectedGoal={x:p[0],y:p[1],base_id:snapshot.editing.base_id,revision:snapshot.editing.revision};
      el('pointMessage').textContent=`已选栅格 (${p[0]}, ${p[1]})，请点击预览；不会驱动车辆。`;editorButtons();draw();return;
    }
    if(drag || !editing || busy || !control.stopped || !snapshot?.editing?.ready || snapshot.layers.static_map?.stale)return;
    const point=cell(event);if(!point)return;
    event.preventDefault();canvas.setPointerCapture(event.pointerId);
    drag={point,id:event.pointerId};
    if(el('mapDrawTool').value==='brush') {draft={cells:[]};paint(point,point);}
    else draft=[point[0],point[1],point[0]+1,point[1]+1];
    draftVersion={base_id:snapshot.editing.base_id,revision:snapshot.editing.revision};
    editorButtons();draw();
  });
  canvas.addEventListener('pointermove',event=>{
    if(!drag || drag.id!==event.pointerId)return;
    event.preventDefault();const p=cell(event);if(!p)return;
    const a=drag.point;
    if(draft.cells) {paint(a,p);drag.point=p;}
    else draft=[Math.min(a[0],p[0]),Math.min(a[1],p[1]),Math.max(a[0],p[0])+1,Math.max(a[1],p[1])+1];
    draw();
  });
  canvas.addEventListener('pointerup',()=>{drag=null;editorButtons();draw();});
  canvas.addEventListener('pointercancel',()=>{drag=null;draft=null;editorButtons();draw();});
  el('mapEditToggle').addEventListener('click',()=>{picking=false;editing=!editing;drag=null;editorButtons();draw();});
  el('mapEditDiscard').addEventListener('click',()=>{draft=null;drag=null;editorButtons();draw();});
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
  function draw() {
    projection=null;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    const layers = snapshot?.layers || {};
    const map = layers.static_map || layers.map;
    if (!map || map.stale) {
      ctx.fillStyle = '#cbd5e1'; ctx.font = '20px sans-serif';
      ctx.fillText(map ? '地图已过期，等待更新' : '等待静态地图或 Nav2 地图', 25, 50);
      return;
    }
    const c = Math.cos(map.yaw), s = Math.sin(map.yaw);
    const wx = map.width * map.resolution, wy = map.height * map.resolution;
    const center = [map.origin[0] + c*wx/2-s*wy/2, map.origin[1]+s*wx/2+c*wy/2];
    const scale = Math.min(canvas.width / wx, canvas.height / wy) * 0.9 * Number(zoom.value);
    projection={scale,wx,wy};
    // Orient the view with the selected map, so the corridor points upward.
    const screen = ([x,y]) => {
      const dx=x-center[0], dy=y-center[1];
      return [canvas.width/2+(c*dx+s*dy)*scale, canvas.height/2-(-s*dx+c*dy)*scale];
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
      ctx.beginPath();ctx.rect(canvas.width/2-wx*scale/2,canvas.height/2-wy*scale/2,wx*scale,wy*scale);ctx.clip();
      ctx.translate(...screen(costs.origin));ctx.rotate(-(costs.yaw-map.yaw));
      ctx.scale(scale*costs.resolution,-scale*costs.resolution);ctx.imageSmoothingEnabled=false;ctx.drawImage(overlay,0,0);ctx.restore();
    }
    if(layers.static_map) {
      const rectScreen=([x0,y0,x1,y1])=>[canvas.width/2+(x0*map.resolution-wx/2)*scale,
        canvas.height/2-(y1*map.resolution-wy/2)*scale,(x1-x0)*map.resolution*scale,(y1-y0)*map.resolution*scale];
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
      const layer=layers[name]; if (!layer || layer.stale || !layer.points.length) continue;
      ctx.strokeStyle=color; ctx.fillStyle=color; ctx.lineWidth=3;
      if (name==='radar') {
        for (const point of layer.points) {const [x,y]=screen(point);ctx.beginPath();ctx.arc(x,y,2.5,0,2*Math.PI);ctx.fill();}
      } else {
        ctx.beginPath(); layer.points.forEach((p,i)=>{const [x,y]=screen(p);if(i)ctx.lineTo(x,y);else ctx.moveTo(x,y);});
        if(name==='footprint')ctx.closePath(); ctx.stroke();
      }
    }
    if (preview?.state === 'ready' && (preview.age || 0) + (performance.now()-previewReceived)/1000 < 30) {
      ctx.save(); ctx.strokeStyle='#06b6d4'; ctx.lineWidth=4; ctx.setLineDash([10,6]); ctx.beginPath();
      preview.points.forEach((p,i)=>{const [x,y]=screen(p);if(i)ctx.lineTo(x,y);else ctx.moveTo(x,y);});
      ctx.stroke();ctx.restore();
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
  async function refresh() {
    try {
      if (!document.hidden) {
        const response=await fetch('/api/obstacle-map',{cache:'no-store',signal:AbortSignal.timeout(3000)});
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
      preview=null;draft=null;drag=null;picking=false;selectedGoal=null;
      window.dispatchEvent(new CustomEvent('nav2-map-snapshot',{detail:null}));
      editorButtons();draw();
      if(cached){ctx.fillStyle='#fbbf24';ctx.font='20px sans-serif';ctx.fillText('缓存底图 · 连接中断，实时图层已隐藏',18,28);}
      status.textContent=`地图连接失败：${error.message}；${cached?'仅显示缓存底图，地图操作已禁用':'等待连接恢复'}`;
    } finally {setTimeout(refresh,500);}
  }
  zoom.addEventListener('input',draw);
  showInflation.addEventListener('change',draw);
  document.getElementById('mapReset').addEventListener('click',()=>{zoom.value='1';draw();});
  document.addEventListener('visibilitychange',()=>{if(document.hidden){snapshot=null;draw();}});
  refresh();
})();
