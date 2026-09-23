(() => {
 const el=id=>document.getElementById(id), enabled=el('wallEnabled'), img=el('wallImage');
 const yellow=el('yellowWallEnabled');
 let vision=null;
 const yellowReady=()=>yellow.checked && vision?.enabled && vision.stopped && vision.wall_revision && vision.wall_count>0;
 const fields=['wallCameraHeight','wallPitch','wallHeight'];
 let url=null, generation=0, active=false, snapshot=null, last=null, retryTimer=null;
 let control={enabled:false,stopped:false};
 function clear(){img.hidden=true;img.removeAttribute('src');if(url)URL.revokeObjectURL(url);url=null;}
 function invalidate(){generation++;last=null;if(retryTimer)clearTimeout(retryTimer);retryTimer=null;clear();}
 function state(){
   const robot=snapshot?.layers?.robot, map=snapshot?.layers?.static_map;
   if((!enabled.checked && !yellowReady()) || document.hidden || !control.enabled || !control.stopped || !robot || robot.stale || !map || map.stale || !snapshot.editing?.ready)return null;
   return {key:JSON.stringify([snapshot.editing.base_id,snapshot.editing.revision,enabled.checked,Boolean(yellowReady()),yellow.checked,vision?.enabled,vision?.wall_revision ?? null,...fields.map(id=>el(id).value)]),position:robot.position.slice(),yaw:robot.yaw};
 }
 function changed(a,b){return !b || a.key!==b.key || Math.hypot(a.position[0]-b.position[0],a.position[1]-b.position[1])>=.01 || Math.abs(Math.atan2(Math.sin(a.yaw-b.yaw),Math.cos(a.yaw-b.yaw)))>=Math.PI/180;}
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
     el('wallMessage').textContent='停车快照 · 红色：手绘禁区；黄色：视觉疑似障碍。墙体或车位变化后更新';
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
