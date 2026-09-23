(() => {
 const el=id=>document.getElementById(id),box=el('inflationEnabled'),slider=el('inflationRadius'),apply=el('inflationApply'),msg=el('inflationMessage');
 let stopped=false,busy=false,loaded=false;
 function controls(){box.disabled=slider.disabled=apply.disabled=busy || !stopped || !loaded;}
 function show(s){box.checked=s.local.enabled;slider.value=s.local.radius;el('inflationRadiusValue').textContent=s.local.radius.toFixed(2)+' 米';}
 async function load(){
  try{const r=await fetch('/api/nav2/inflation',{signal:AbortSignal.timeout(10000)});const s=await r.json();if(!r.ok)throw Error(s.detail || '读取失败');show(s);loaded=true;stopped=s.stopped;if(!s.consistent)msg.textContent='局部与全局设置不一致，请强停后重新应用';}
  catch(e){msg.textContent=e.message+'；点击此提示重试';}finally{controls();}
 }
 slider.addEventListener('input',()=>{el('inflationRadiusValue').textContent=Number(slider.value).toFixed(2)+' 米（未应用）';});
 box.addEventListener('change',()=>{msg.textContent='设置尚未应用，请点击应用缓冲设置';});
 window.addEventListener('nav2-map-control',e=>{stopped=e.detail.stopped && e.detail.enabled;controls();});
 msg.addEventListener('click',()=>{if(!busy)load();});
 apply.addEventListener('click',async()=>{
  busy=true;controls();msg.textContent='正在同步两张代价地图…';
  try{
   const r=await fetch('/api/nav2/inflation',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({enabled:box.checked,radius:Number(slider.value)}),signal:AbortSignal.timeout(15000)});
   const s=await r.json();if(!r.ok)throw Error(s.detail || '应用失败');show(s);
   msg.textContent=s.local.enabled?'缓冲区已启用：'+s.local.radius.toFixed(2)+' 米，请重新预览路径':'膨胀缓冲区已关闭，障碍物和车身碰撞检查保留；请重新预览路径';
  }catch(e){msg.textContent=e.message;}finally{busy=false;controls();}
 });
 load();
})();
