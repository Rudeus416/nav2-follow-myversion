// 【内容标注】用途：紫色缓冲开关与半径滑块。
// 对应用户需求：R12 R14（原话及追溯边界见 nav2/CODE_GUIDE.md）。
// 添加/修改逻辑：区分未应用设置与后端确认状态，强停时提交半径并更新提示。
// 本次仅加注释；需求关联不是精确创建/提交记录。
(() => {
 const el=id=>document.getElementById(id),box=el('inflationEnabled'),slider=el('inflationRadius'),apply=el('inflationApply'),msg=el('inflationMessage');
 // Keep shared HTML unchanged; label belongs to this Nav2 extension.
 const labelText=box.parentElement.lastChild;
 if(labelText?.nodeType===3)labelText.textContent=' 启用紫色禁入缓冲（整车不得进入）';
 let stopped=false,busy=false,loaded=false;
 // 【职责 / R12 R14】controls：依操作状态启禁控件。
 function controls(){box.disabled=slider.disabled=apply.disabled=busy || !stopped || !loaded;}
 // 【职责 / R12 R14】show：显示服务端确认后的参数。
 function show(s){msg.textContent='整车避障已启用；紫色禁入半径 '+(s.local.enabled?s.local.radius:0).toFixed(2)+' 米；0 米仍检查完整车身';box.checked=s.local.enabled;slider.value=s.local.radius;el('inflationRadiusValue').textContent=s.local.radius.toFixed(2)+' 米';}
 // 【职责 / R12 R14】load：读取后端参数并更新确认状态。
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
   msg.textContent=(s.local.enabled?'紫色禁入半径：'+s.local.radius.toFixed(2)+' 米':'紫色缓冲已关闭')+'；完整车身及 5 厘米边距检查保留，请重新预览路径';
  }catch(e){msg.textContent=e.message;}finally{busy=false;controls();}
 });
 load();
})();
