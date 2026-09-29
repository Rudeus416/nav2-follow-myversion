# 【内容标注】用途：语义工作的单线程最新引用分发器。
# 对应用户需求：R37（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：主视觉 tick 只交换引用；模型启动、缩放、收取和关闭均由独立 owner 串行执行。
"""Keep optional semantic processing outside the depth safety consumer."""
import math
import threading
import time


# 【职责 / R37】SemanticDispatch：独占语义 worker，用单线程处理最新引用并缓存结果。
class SemanticDispatch:
    """R37: one owner, one replaceable offer, and a cached result/status.

    The supplied worker belongs exclusively to this dispatcher. Public methods
    never call its update/status/close methods. A blocked update can therefore
    delay semantic cleanup, but cannot block depth ticks or race worker.close().
    """

    # 【职责 / R37】__init__：建立单 owner、可替换输入和代次隔离状态。
    def __init__(self, worker=None, poll_interval=.05):
        if worker is None:
            from nav2.semantic_obstacles import SemanticWorker
            worker=SemanticWorker()
        self._worker=worker
        self._model_enabled=bool(getattr(worker,'enabled',True))
        self._poll_interval=max(.001,float(poll_interval))
        self._lock=threading.Lock()
        self._wake=threading.Event()
        self._closed=False
        self._enabled=True
        self._generation=0
        self._reset_requested=False
        self._offered=None
        self._packet=None
        self._status={'enabled':self._model_enabled,
                      'message':getattr(worker,'message','等待语义帧'),
                      'fresh':False,'objects':[]}
        self._thread=threading.Thread(target=self._run,
            name='nav2-semantic-owner',daemon=True)
        self._thread.start()

    # 【职责 / R37】offer：只交换最新帧引用，不在视觉线程运行模型或复制大图。
    def offer(self, record):
        """R37: retain only the newest offered reference, without copying images."""
        if record is None:
            return
        with self._lock:
            if self._closed or not self._enabled or not self._model_enabled:
                return
            self._offered=record
            packet=self._packet
            if packet is not None and self._context(packet[0])!=self._context(record):
                self._packet=None
                self._status=dict(self._status,fresh=False,objects=[])
            self._wake.set()

    # 【职责 / R37】snapshot：返回仍与当前流匹配且新鲜的同帧语义包。
    def snapshot(self):
        """Return the original source record and its fresh semantic result."""
        with self._lock:
            packet=self._packet
            if self._closed or not self._enabled or not self._fresh(packet):
                return None
            return packet

    # 【职责 / R37】status：无等待读取缓存诊断，过期结果不暴露物体。
    def status(self):
        """R37: UI reads a cache; it never waits for the semantic worker."""
        with self._lock:
            status=dict(self._status)
            fresh=self._enabled and not self._closed and self._fresh(self._packet)
            status['fresh']=fresh
            if not fresh:
                status['objects']=[]
            return status

    # 【职责 / R37】set_enabled：切换时立即失效旧结果，并由 owner 线程重置 worker。
    def set_enabled(self, enabled):
        """Invalidate results immediately; reset the worker on its owner thread."""
        enabled=bool(enabled)
        with self._lock:
            if self._closed or enabled==self._enabled:
                return
            self._enabled=enabled
            self._generation+=1
            self._offered=None
            self._packet=None
            # Even a quick off/on must close the old pending inference before
            # any new generation can publish. This flag survives re-enabling.
            if not enabled:
                self._reset_requested=True
            self._status={'enabled':enabled and self._model_enabled,
                          'message':('YOLOE 已关闭' if not self._model_enabled else
                                     '等待语义帧' if enabled else 'YOLOE 已暂停'),
                          'fresh':False,'objects':[]}
            self._wake.set()

    # 【职责 / R37】close：立即撤销结果；可选等待被限制在短时且不关闭他人资源。
    def close(self, timeout=0.):
        """R37: signal shutdown immediately; any optional join is bounded.

        The default does not join, so callers can safely signal shutdown even
        while holding their own locks. Only the owner invokes worker.close().
        """
        with self._lock:
            self._closed=True
            self._generation+=1
            self._offered=None
            self._packet=None
            self._status={'enabled':False,'message':'YOLOE 已关闭',
                          'fresh':False,'objects':[]}
            self._wake.set()
        if timeout>0 and self._thread is not threading.current_thread():
            self._thread.join(timeout=min(float(timeout),.2))

    # 【职责 / R37】_context：提取配置版本和相机流作为语义身份。
    @staticmethod
    def _context(record):
        return record.config_version,record.frame.stream_epoch

    # 【职责 / R37】_fresh：以原采集时间判断语义包是否仍在 1.2 秒期限内。
    @staticmethod
    def _fresh(packet):
        if packet is None:
            return False
        source_at=packet[0].frame.source_at
        age=time.monotonic()-source_at
        return math.isfinite(source_at) and 0<=age<=1.2

    # 【职责 / R37】_run：串行更新/收包/重置，积压时只处理最新引用。
    def _run(self):
        record=None
        record_generation=None
        try:
            while True:
                # Clear before reading the mailbox: an offer during update
                # must wake the next pass, without accumulating queued work.
                self._wake.clear()
                with self._lock:
                    if self._closed:
                        break
                    reset=self._reset_requested
                    self._reset_requested=False
                    enabled=self._enabled
                    generation=self._generation
                    offered=self._offered
                    self._offered=None
                if reset:
                    record=None
                    record_generation=None
                    closed=self._close_worker()
                    # Re-read after close: it can be slow, and off/on or a new
                    # offer may have happened while the old process shut down.
                    with self._lock:
                        if not closed:
                            self._reset_requested=True
                        if (not self._closed and self._enabled
                                and self._generation==generation
                                and self._offered is None):
                            self._offered=offered
                    if not closed:
                        self._wake.wait(self._poll_interval)
                    continue
                if generation!=record_generation:
                    record=None
                    record_generation=generation
                if enabled and offered is not None:
                    record=offered
                if enabled and record is not None:
                    try:
                        packet=self._worker.update(record)
                        status=self._worker.status()
                    except Exception as exc:
                        packet=None
                        status={'enabled':self._model_enabled,
                                'message':'YOLOE 接入失败：'+str(exc),
                                'fresh':False,'objects':[]}
                    with self._lock:
                        if (not self._closed and self._enabled
                                and generation==self._generation):
                            latest=self._offered if self._offered is not None else record
                            if packet is not None and self._context(packet[0])!=self._context(latest):
                                packet=None
                            self._packet=packet
                            self._status=status
                # Poll the retained record even without new offers. Worker
                # update drains same-frame results and owns dedup/freshness.
                self._wake.wait(self._poll_interval)
        finally:
            self._close_worker()

    # 【职责 / R37】_close_worker：只有 owner 调用实际关闭，失败转为可见诊断。
    def _close_worker(self):
        try:
            self._worker.close()
            return True
        except Exception as exc:
            # Optional semantics cannot kill or block the depth safety loop.
            with self._lock:
                if not self._closed:
                    self._status={'enabled':False,
                                  'message':'YOLOE 关闭失败：'+str(exc),
                                  'fresh':False,'objects':[]}
            return False
