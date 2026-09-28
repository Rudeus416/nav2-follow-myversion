// 【内容标注】用途：同窗红黄虚拟墙网页显示。
// 对应用户需求：R05 R06（原话及追溯边界见 nav2/CODE_GUIDE.md）。
// 添加/修改逻辑：墙体/车位或设置改变时刷新；请求代际丢弃过期响应；释放图像资源。
// 本次仅加注释；需求关联不是精确创建/提交记录。
(() => {
 const el=id=>document.getElementById(id), enabled=el('wallEnabled'), img=el('wallImage');
 const yellow=el('yellowWallEnabled');
 let vision=null;
 const yellowReady=()=>yellow.checked && vision?.enabled && vision.stopped && vision.wall_revision && (vision.wall_count>0 || (vision.semantic?.fresh && vision.semantic.objects?.length>0));
 const fields=['wallCameraHeight','wallPitch','wallHeight'];
 let url=null, generation=0, active=false, snapshot=null, last=null, retryTimer=null;
 let control={enabled:false,stopped:false};
 // 【职责 / R05 R06】clear：隐藏图像并释放对象 URL。
 function clear(){img.hidden=true;img.removeAttribute('src');if(url)URL.revokeObjectURL(url);url=null;}
 // 【职责 / R05 R06】invalidate：递增请求代际，清掉旧快照与等待任务。
 function invalidate(){generation++;last=null;if(retryTimer)clearTimeout(retryTimer);retryTimer=null;clear();}
 // 【职责 / R05 R06】state：整理当前预览输入状态。
 function state(){
   const robot=snapshot?.layers?.robot, map=snapshot?.layers?.static_map;
   if((!enabled.checked && !yellowReady()) || document.hidden || !control.enabled || !control.stopped || !robot || robot.stale || !map || map.stale || !snapshot.editing?.ready)return null;
   return {key:JSON.stringify([snapshot.editing.base_id,snapshot.editing.revision,enabled.checked,Boolean(yellowReady()),yellow.checked,vision?.enabled,vision?.wall_revision ?? null,vision?.semantic_frame,vision?.reason,vision?.diagnostics?.camera_only,...fields.map(id=>el(id).value)]),position:robot.position.slice(),yaw:robot.yaw};
 }
 // 【职责 / R05 R06】changed：比较墙体或车位变化以决定是否刷新。
 function changed(a,b){return !b || a.key!==b.key || Math.hypot(a.position[0]-b.position[0],a.position[1]-b.position[1])>=.01 || Math.abs(Math.atan2(Math.sin(a.yaw-b.yaw),Math.cos(a.yaw-b.yaw)))>=Math.PI/180;}
 // 【职责 / R05 R06】refresh：异步刷新本面板数据，处理过期响应及资源清理。
 async function refresh(){
   el('wallControls').hidden=!(enabled.checked || yellow.checked);
   const next=state();
   if(!next){if(last || url)invalidate();return;}
   if(active || retryTimer || !changed(next,last))return;
   active=true;const token=generation;
   try {
     if(fields.some(id=>!el(id).value || !el(id).checkValidity()))throw new Error('请填写有效的相机高度、俯角和墙高');
     const params=new URLSearchParams({red:String(enabled.checked),vision:String(Boolean(yellow.checked && vision?.enabled)),camera_height:el(fields[0]).value,pitch_down:el(fields[1]).value,wall_height:el(fields[2]).value});
     const r=await fetch('/api/nav2/virtual-wall?'+params,{cache:'no-store',signal:AbortSignal.timeout(5000)});
     if(!r.ok){const error=await r.json();throw new Error(typeof error.detail==='string'?error.detail:'预览参数无效');}
     const blob=await r.blob();const current=state();
     if(token!==generation || !current || changed(current,next))return;
     clear();url=URL.createObjectURL(blob);img.src=url;img.hidden=false;last=next;
     const count=Number(r.headers.get('X-Yellow-Wall-Count') || 0);
     el('wallMessage').textContent='停车快照 · 红色：手绘禁区；黄色：视觉疑似障碍。'+(yellow.checked ? (count ? (r.headers.get('X-Yellow-Camera-Only')==='1'?'黄色候选在地图外，仅供相机预览，不写入导航地图':'黄色候选已用于导航地图') : '当前无可绘制的黄色候选；'+(vision?.reason || '等待视觉数据')) : '黄色墙未开启');
     if(Number(r.headers.get('X-Semantic-Count'))>0)el('wallMessage').textContent+='；青色轮廓与英文标签为 YOLOE 识别，黄色墙仍需有效测距';
   }catch(error){if(token===generation){
     clear();last=null;el('wallMessage').textContent=error.message+'；等待数据恢复后自动重试';
     retryTimer=setTimeout(()=>{retryTimer=null;refresh();},1000);
   }}
   finally{active=false;const current=state();if(!retryTimer && current && changed(current,last))refresh();}
 }
 enabled.addEventListener('change',()=>{invalidate();el('wallControls').hidden=!enabled.checked;refresh();});
 yellow.addEventListener('change',()=>{invalidate();refresh();});
 window.addEventListener('nav2-vision-state',event=>{vision=event.detail;yellow.disabled=!vision?.enabled || !vision.stopped;refresh();});
 for(const id of fields)el(id).addEventListener('change',()=>{invalidate();refresh();});
 window.addEventListener('nav2-map-snapshot',event=>{snapshot=event.detail;refresh();});
 window.addEventListener('nav2-map-control',event=>{control=event.detail;refresh();});
 document.addEventListener('visibilitychange',()=>{invalidate();refresh();});
 window.addEventListener('beforeunload',clear);
})();
