// 【内容标注】用途：视觉障碍选择与估距校验界面。
// 对应用户需求：R07 R10（原话及追溯边界见 nav2/CODE_GUIDE.md）。
// 添加/修改逻辑：开关视觉层、显示诊断、提交停车图像选点和实测距离校验。
// 本次仅加注释；需求关联不是精确创建/提交记录。
(() => {
 const box=document.getElementById('navVisionEnabled'),message=document.getElementById('navVisionToggleMessage');
 // Nav2-only UI: do not edit the shared camera page or its original controls.
 const panel=document.createElement('section');
 panel.innerHTML=`<details><summary>Nav2 视觉距离校准（需强停）</summary>
 <p>获取画面后，点击最近障碍物内部表面；测量相机到该点的直线距离。不要点空隙、边缘或背景。画面有效期 60 秒，校准时保持小车及物体不动。</p>
 <button type="button" data-capture>获取校准画面</button>
 <div style="position:relative;width:100%;max-width:800px" data-view hidden><img data-image style="display:block;width:100%;cursor:crosshair" alt="点击实测障碍表面"><span data-mark hidden style="position:absolute;color:#ff3030;font-size:28px;pointer-events:none;transform:translate(-50%,-50%)">＋</span></div>
 <label>相机到选点的实测距离（米）<input data-distance type="number" min="0.15" max="5" step="0.01" placeholder="请填写实测值"></label>
 <button type="button" data-apply disabled>应用 Nav2 测距校准</button><p data-result></p>
 <p>只影响新增导航视觉障碍，不影响原人物测距；单点比例不保证所有物体准确，重启后恢复启动比例。</p></details>`;
 message.insertAdjacentElement('afterend',panel);
 const find=s=>panel.querySelector(s),capture=find('[data-capture]'),apply=find('[data-apply]'),image=find('[data-image]'),result=find('[data-result]');
 let sample=null,point=null,calibrating=false,stopped=false;
 const calibrationControls=()=>{capture.disabled=!stopped || calibrating;apply.disabled=!stopped || calibrating || !sample || !point;};
 image.addEventListener('click',e=>{const rect=image.getBoundingClientRect();point={u:(e.clientX-rect.left)/rect.width,v:(e.clientY-rect.top)/rect.height};const mark=find('[data-mark]');mark.hidden=false;mark.style.left=(point.u*100)+'%';mark.style.top=(point.v*100)+'%';calibrationControls();});
 capture.addEventListener('click',async()=>{calibrating=true;sample=null;point=null;calibrationControls();find('[data-view]').hidden=true;find('[data-mark]').hidden=true;
  try{const r=await fetch('/api/nav2/vision-calibration',{cache:'no-store',signal:AbortSignal.timeout(5000)});const s=await r.json();if(!r.ok)throw Error(s.detail);sample=s;image.src=s.image;find('[data-view]').hidden=false;result.textContent='请选择物体表面，并输入该点实测距离';}
  catch(e){result.textContent=e.message;}finally{calibrating=false;calibrationControls();}});
 apply.addEventListener('click',async()=>{if(!sample || !point)return;const input=find('[data-distance]');if(!input.value || !input.checkValidity()){result.textContent='请填写有效实测距离';return;}calibrating=true;calibrationControls();
  try{const r=await fetch('/api/nav2/vision-calibration',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({token:sample.token,...point,distance:Number(input.value)}),signal:AbortSignal.timeout(5000)});const s=await r.json();if(!r.ok)throw Error(s.detail);result.textContent=s.message+' 原估计 '+s.raw_range.toFixed(2)+' 米，比例 '+s.scale.toFixed(3);sample=null;point=null;}
  catch(e){result.textContent=e.message;}finally{calibrating=false;calibrationControls();}});
 calibrationControls();
 let busy=false,confirmed=false;
 // 【职责 / R07 R10】update：拉取视觉状态并更新开关/诊断。
 async function update(){
  if(busy || document.hidden)return;
  try{
   const r=await fetch('/api/nav2/vision-enabled',{cache:'no-store',signal:AbortSignal.timeout(3000)});
   if(!r.ok)throw new Error('请启用 Nav2 并重启服务以加载网页开关');
   const s=await r.json();if(busy)return;confirmed=s.enabled;box.checked=s.enabled;box.disabled=!s.stopped;stopped=s.stopped;calibrationControls();
   if(s.enabled)message.textContent='视觉筛选：'+(s.reason || '等待候选')+'；相机墙格 '+s.wall_count+'，地图墙格 '+(s.map_wall_count ?? s.wall_count);
   if(s.enabled && s.semantic)message.textContent+='；'+s.semantic.message+(s.semantic.fresh?'':'（无新鲜识别，仍用深度障碍）');
   window.dispatchEvent(new CustomEvent('nav2-vision-state',{detail:s}));
  }catch(e){if(!busy){box.disabled=true;stopped=false;calibrationControls();message.textContent=e.message;window.dispatchEvent(new CustomEvent('nav2-vision-state',{detail:null}));}}
 }
 box.addEventListener('change',async()=>{
  const enabled=box.checked;busy=true;box.disabled=true;message.textContent='正在同步导航视觉图层…';
  try{
   const r=await fetch('/api/nav2/vision-enabled',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({enabled}),signal:AbortSignal.timeout(12000)});
   const s=await r.json();if(!r.ok)throw new Error(s.detail || '切换失败');
   confirmed=s.enabled;box.checked=confirmed;
   message.textContent=confirmed?'视觉障碍检测已开启，等待新鲜数据；仍保持强停':'视觉障碍检测已关闭，地图墙体及禁区仍生效；仍保持强停';
  }catch(e){box.checked=confirmed;message.textContent=e.message;}
  finally{busy=false;update();}
 });
 box.disabled=true;update();setInterval(update,2000);
})();
