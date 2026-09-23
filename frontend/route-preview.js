(() => {
 const el=id=>document.getElementById(id), image=el('visionPreviewImage'), toggle=el('visionPreviewEnabled');
 let url=null, generation=0;
 function clear(){image.hidden=true;image.removeAttribute('src');if(url)URL.revokeObjectURL(url);url=null;}
 for(const id of ['visionPreviewEnabled','visionHeight','visionPitch','visionScale'])el(id).addEventListener('change',()=>{generation++;clear();});
 window.addEventListener('nav2-map-control',e=>{if(!e.detail.enabled || !e.detail.stopped){toggle.checked=false;generation++;clear();}});
 document.addEventListener('visibilitychange',()=>{generation++;clear();});
 async function refresh(){
  const token=generation;
  try {
   if(!toggle.checked || document.hidden)return;
   for(const id of ['visionHeight','visionPitch','visionScale'])if(!el(id).value || !el(id).checkValidity())throw new Error('请填写有效预览参数');
   const q=new URLSearchParams({camera_height:el('visionHeight').value,pitch_down:el('visionPitch').value,depth_scale:el('visionScale').value});
   const r=await fetch('/api/nav2/vision-obstacle-preview?'+q,{cache:'no-store',signal:AbortSignal.timeout(5000)});
   if(!r.ok){const e=await r.json();throw new Error(typeof e.detail==='string'?e.detail:'预览参数无效');}
   const blob=await r.blob();if(token!==generation || !toggle.checked || document.hidden)return;
   clear();url=URL.createObjectURL(blob);image.src=url;image.hidden=false;
   const distance=r.headers.get('X-Candidate-Distance');
   el('visionPreviewMessage').textContent=`${distance?'疑似障碍估计距离 '+distance+' 米':'本帧未检出符合条件的区域（不代表安全）'}；图像延迟 ${r.headers.get('X-Frame-Age')} 秒。尚未标定，不用于行驶放行。`;
  }catch(e){if(token===generation){clear();el('visionPreviewMessage').textContent=e.message;}}
  finally{setTimeout(refresh,1000);}
 }
 window.addEventListener('beforeunload',clear);refresh();
})();
