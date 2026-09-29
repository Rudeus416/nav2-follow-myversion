# 【内容标注】用途：Nav2 自有语义子进程的资源限额。
# 对应用户需求：R38（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：只降低自有子进程优先级；模型后端重建后恢复单线程，避免覆盖原人物/深度模型设置。
"""Limit the optional child, including native pools already loaded by spawn."""
import multiprocessing as mp
import os


class SemanticRuntime:
    """Created only inside our multiprocessing child, never the camera process."""

    def __init__(self, torch_module=None, cv2_module=None, pool_limiter=None):
        if mp.parent_process() is None:
            raise RuntimeError('语义资源限制只能在自有子进程设置')
        self._lower_priority()
        # spawn may import the original main module before calling our target.
        # Environment values cover future imports; threadpoolctl below covers
        # BLAS/OpenMP pools that were already loaded in this child.
        for name in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS',
                     'NUMEXPR_NUM_THREADS','NUMEXPR_MAX_THREADS','VECLIB_MAXIMUM_THREADS'):
            os.environ[name]='1'
        if torch_module is None:
            import torch as torch_module
        if cv2_module is None:
            import cv2 as cv2_module
        if pool_limiter is None:
            from threadpoolctl import threadpool_limits as pool_limiter
        self.torch=torch_module
        self.cv2=cv2_module
        self._pool_limiter=pool_limiter
        self._limited_predictors={}
        self.before_inference=None
        # Inter-op has a once-before-work requirement. Do not reset it every
        # prediction, unlike the intra-op limit overridden by select_device().
        if self.torch.get_num_interop_threads()!=1:
            self.torch.set_num_interop_threads(1)
        self.apply()

    @staticmethod
    def _lower_priority():
        # Linux nice is per-thread. Lower the calling thread first, then pools
        # imported during spawn; subsequently created threads inherit nice>=10.
        priority=os.getpriority(os.PRIO_PROCESS,0)
        if priority<10:os.setpriority(os.PRIO_PROCESS,0,10)
        for task in os.listdir('/proc/self/task'):
            try:
                priority=os.getpriority(os.PRIO_PROCESS,int(task))
                if priority<10:os.setpriority(os.PRIO_PROCESS,int(task),10)
            except ProcessLookupError:
                pass  # A native helper thread may have finished meanwhile.

    def apply(self):
        """Reapply after backend setup, before any warmup/forward inference."""
        self.torch.set_num_threads(1)
        self.cv2.setNumThreads(1)
        # Keep the limiter alive throughout the child's lifetime. There is no
        # context exit restoring wider pools between requests.
        self._pool_limits=self._pool_limiter(limits=1)

    def predictor_type(self, base):
        """Preserve the task-specific predictor and bound every backend rebuild."""
        if base not in self._limited_predictors:
            runtime=self
            class LimitedPredictor(base):
                def setup_model(self, *args, **kwargs):
                    result=super().setup_model(*args, **kwargs)
                    # Ultralytics select_device('cpu') resets torch to its
                    # machine-derived NUM_THREADS inside the call above.
                    runtime.apply()
                    return result

                def inference(self, *args, **kwargs):
                    # Source setup/preprocessing can itself take time on the
                    # first image. The optional owner's guard runs immediately
                    # before forward, after all that work has completed.
                    if runtime.before_inference is not None:
                        runtime.before_inference(self)
                    return super().inference(*args, **kwargs)
            self._limited_predictors[base]=LimitedPredictor
        return self._limited_predictors[base]

    def model(self, model_type, model_path):
        """Wrap only this child-owned YOLOE; do not patch library globals.

        Installed YOLOE.predict consumes its predictor argument for visual
        prompts and drops it for prompt-free models. Overriding _smart_load is
        therefore necessary for both first setup and later backend rebuilds.
        """
        runtime=self
        class LimitedSemanticModel(model_type):
            def _smart_load(self, key):
                component=super()._smart_load(key)
                return runtime.predictor_type(component) if key=='predictor' else component
        return LimitedSemanticModel(model_path)
