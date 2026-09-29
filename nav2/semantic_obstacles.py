# 【内容标注】用途：独立语义物体检测工作进程。
# 对应用户需求：R09 R08 R38（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：提取 YOLO 实例区域辅助最近候选选择；独立模型/进程，不替换原人物检测链路。
# R35 阻止重复/过期帧提交；只在成功入队后推进去重和节流状态。
"""Independent YOLOE worker for Nav2; never shares the person tracker/model."""
import multiprocessing as mp
import math
import logging
import os
from pathlib import Path
import queue
import time
import cv2
import numpy as np

OBSTACLE_LABELS = {'handcart','cart','trolley','shopping cart','chair','table','desk',
                   'cabinet','file cabinet','box','storage box','suitcase','person',
                   'stool','shelf','sofa','bed','bench'}

MODEL = Path(__file__).resolve().parent / 'models/yoloe-26n-seg-pf.pt'


# 【职责 / R09 R08】detections：将实例检测输出转换为原图归一化多边形。
def detections(result):
    """Normalized polygons preserve original image geometry (no letterbox mask resize)."""
    if result.masks is None or result.boxes is None:
        return []
    found=[]
    for polygon,cls,confidence in zip(result.masks.xyn,result.boxes.cls,result.boxes.conf):
        if len(polygon)<3 or not np.isfinite(polygon).all():continue
        found.append({'label':result.names[int(cls)],'confidence':float(confidence),
                      'polygon':np.clip(polygon,0,1).tolist()})
    return found


# 【职责 / R09 R08】instance_masks：整理语义区域供深度候选选择使用。
def instance_masks(objects, width=80, height=60):
    masks=[]
    for obj in objects:
        points=np.asarray(obj['polygon'],float)
        if points.ndim!=2 or points.shape[1]!=2 or len(points)<3 or not np.isfinite(points).all():continue
        pixels=np.minimum(np.floor(np.clip(points,0,1)*[width,height]),[width-1,height-1]).astype(np.int32)
        mask=np.zeros((height,width),np.uint8);cv2.fillPoly(mask,[pixels],1)
        masks.append((obj['label'],mask.astype(bool)))
    return masks


# 【职责 / R09 R08】_worker：独立进程加载/运行语义模型，避免覆盖人物检测模型。
def _worker(requests, responses, model_path, device):
    # All global torch settings and model state stay in this separate process.
    try:
        from nav2.semantic_runtime import SemanticRuntime
        runtime=SemanticRuntime()
        torch=runtime.torch
        from ultralytics import YOLOE
        model=runtime.model(YOLOE,model_path)
        from nav2.semantic_request import RequestFreshness,SemanticRequestExpired
        freshness=RequestFreshness()
        runtime.before_inference=freshness  # R38: recheck after preprocessing
        model.add_callback('on_predict_start',freshness)
        device=('0' if torch.cuda.is_available() else 'cpu') if device=='auto' else device
        allowed=[i for i,name in model.names.items() if name in OBSTACLE_LABELS]
        if not allowed:raise ValueError('模型词表不含配置的实体障碍类别')
        responses.put(('ready',None,None))
        while True:
            job=requests.get()
            if job is None:return
            token,image,source_at=job
            try:
                freshness.begin(source_at)
                result=model.predict(image,imgsz=640,device=device,conf=.20,verbose=False,
                                     max_det=30,retina_masks=True,classes=allowed)[0]
                responses.put(('result',token,detections(result)))
            except SemanticRequestExpired as exc:responses.put(('dropped',token,str(exc)))
            except Exception as exc:responses.put(('error',token,str(exc)))
    except Exception as exc:responses.put(('error',None,str(exc)))


# 【职责 / R09 R08】SemanticWorker：独立语义物体检测工作进程的状态封装；各方法职责见下方标注。
class SemanticWorker:
    # 【职责 / R09 R08】__init__：初始化本类依赖与状态；副作用以原初始化语句为准。
    def __init__(self):
        self.enabled=os.environ.get('NAV2_VISION_YOLO','1')=='1'
        self.path=Path(os.environ.get('NAV2_YOLO_MODEL',str(MODEL))).resolve()
        self.process=None;self.pending=None;self.latest=None;self.last_offer=0.
        self.last_submitted=None
        self.attempted=False;self.message='等待启用视觉障碍检测' if self.enabled else 'YOLOE 已关闭'
        from nav2.semantic_budget import SemanticBudget
        self.budget=SemanticBudget();self.resource_budget={}
        self._budget_paused=None

    # 【职责 / R09 R08】update：提交或读取异步语义结果，保持帧关联。
    def update(self, record):
        if not self.enabled:return None
        now=time.monotonic()
        if self.process is None and not self.attempted and not self.path.is_file():
            self.attempted=True
            self.message='YOLOE 权重缺失：'+str(self.path);return None
        # R38：只减自有附加语义工作量。预算暂停不卸载模型，仍收取在途响应。
        self.resource_budget=self.budget.observe(record,now)
        paused=self.resource_budget['paused']
        if paused!=self._budget_paused:
            self._budget_paused=paused
            logging.getLogger(__name__).warning('附加语义预算：%s；%s',
                '暂停新任务，深度避障继续' if paused else '允许新任务',self.resource_budget)
        if self.process is None and not self.attempted and not paused:
            self.attempted=True
            ctx=mp.get_context('spawn')
            self.requests=ctx.Queue(maxsize=1);self.responses=ctx.Queue(maxsize=2)
            self.process=ctx.Process(target=_worker,args=(self.requests,self.responses,str(self.path),os.environ.get('NAV2_YOLO_DEVICE','cpu')),daemon=True)
            self.process.start();self.message='YOLOE 正在加载'
        if self.process is None:return None
        while True:
            try:kind,token,data=self.responses.get_nowait()
            except queue.Empty:break
            if kind=='ready':self.message='YOLOE 已加载，等待识别'
            elif kind=='error':self.message='YOLOE 失败：'+data;self.pending=None;self.latest=None
            elif kind=='dropped' and self.pending is not None and token==self.pending[0]:
                self.pending=None;self.message=str(data)
            elif kind=='result' and self.pending is not None and token==self.pending[0]:
                self.latest=(self.pending[1],data);self.pending=None
                self.message='YOLOE 本帧未识别到物体' if not data else 'YOLOE：'+', '.join(dict.fromkeys(x['label'] for x in data))
        if not self.process.is_alive():self.message='YOLOE 子进程退出，请重启视觉服务';return None
        # R35: polling is not a new observation. Never spend inference on an
        # already submitted/expired frame. Still drain responses above so stale
        # inputs cannot leave a completed request permanently marked in flight.
        if record is None:return None
        now=time.monotonic()
        token=(record.config_version,record.frame.stream_epoch,record.frame.frame_id)
        source_at=record.frame.source_at
        previous=self.last_submitted
        new_frame=(previous is None or token[:2]!=previous[0][:2]
                   or (token[2]>previous[0][2] and source_at>previous[1]))
        if (not paused and self.pending is None and now-self.last_offer>=.5 and new_frame
                and math.isfinite(source_at) and 0<=now-source_at<=1.2):
            image=record.frame.image
            small=cv2.resize(image,(640,round(image.shape[0]*640/image.shape[1])))
            # Resizing can take time on a loaded board. Check again before offer.
            submitted_at=time.monotonic()
            if 0<=submitted_at-source_at<=1.2:
                try:self.requests.put_nowait((token,small,source_at))
                except queue.Full:pass
                else:
                    self.pending=(token,record);self.last_offer=submitted_at
                    self.last_submitted=(token,source_at)
        if self.latest and self.latest[0].config_version==record.config_version and self.latest[0].frame.stream_epoch==record.frame.stream_epoch and now-self.latest[0].frame.source_at<=1.2:
            return self.latest
        return None

    # 【职责 / R09 R08】status：报告语义工作状态。
    def status(self):
        now=time.monotonic()
        latest=self.latest
        fresh=bool(latest and now-latest[0].frame.source_at<=1.2)
        budget=dict(getattr(self,'resource_budget',{}))
        # R38：预算提示附在模型状态后，暂停时也保留加载/退出/失败原因。
        message=self.message
        if budget.get('paused'):message+='；'+budget['reason']+'；深度避障继续'
        return {'enabled':self.enabled,'message':message,'fresh':fresh,'resource_budget':budget,
                'objects':[{k:o[k] for k in ('label','confidence')} for o in latest[1]] if fresh else []}

    # 【职责 / R09 R08 R37】close：串行清理正常运行或部分初始化的语义进程。
    def close(self):
        self.pending=None;self.latest=None
        self.last_submitted=None;self.last_offer=0.
        # R37: retain ownership until the old process has actually stopped.
        # A failed cleanup must not permit update() to start a second process.
        self.attempted=True
        process=self.process
        if process is not None:
            if process.is_alive():process.terminate()
            # start() can raise after assigning self.process. Joining an
            # unstarted Process raises AssertionError and used to prevent reset.
            if process.pid is not None:process.join(timeout=2.)
            if process.is_alive():
                self.message='YOLOE 子进程仍在退出，等待清理重试'
                raise RuntimeError(self.message)
            process.close()
            self.process=None
        # Queue construction can fail between requests and responses. Close
        # every queue that exists, even if cleanup of the other one fails.
        failure=None
        for name in ('requests','responses'):
            channel=getattr(self,name,None)
            if channel is None:continue
            try:channel.cancel_join_thread()
            except Exception as exc:
                if failure is None:failure=exc
            try:channel.close()
            except Exception as exc:
                if failure is None:failure=exc
            else:setattr(self,name,None)
        if failure is not None:
            raise RuntimeError('YOLOE 队列清理失败：'+str(failure)) from failure
        self.attempted=False
        from nav2.semantic_budget import SemanticBudget
        self.budget=SemanticBudget();self.resource_budget={};self._budget_paused=None
