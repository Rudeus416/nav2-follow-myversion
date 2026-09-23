(() => {
 const box=document.getElementById('navVisionEnabled'),message=document.getElementById('navVisionToggleMessage');
 let busy=false,confirmed=false;
 async function update(){
  if(busy || document.hidden)return;
  try{
   const r=await fetch('/api/nav2/vision-enabled',{cache:'no-store',signal:AbortSignal.timeout(3000)});
   if(!r.ok)throw new Error('请启用 Nav2 并重启服务以加载网页开关');
   const s=await r.json();if(busy)return;confirmed=s.enabled;box.checked=s.enabled;box.disabled=!s.stopped;
   window.dispatchEvent(new CustomEvent('nav2-vision-state',{detail:s}));
  }catch(e){if(!busy){box.disabled=true;message.textContent=e.message;window.dispatchEvent(new CustomEvent('nav2-vision-state',{detail:null}));}}
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
