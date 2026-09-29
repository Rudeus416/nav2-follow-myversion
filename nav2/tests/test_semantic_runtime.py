# 【内容标注】用途：自有语义子进程资源限制的离线回归。
# 对应用户需求：R38（原话及追溯边界见 nav2/CODE_GUIDE.md）。
# 添加/修改逻辑：假后端在setup重置7线程，验证首次和重建推理前恢复1；不启动模型/ROS/底盘。
import os
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

from nav2.semantic_runtime import SemanticRuntime


class SemanticRuntimeTests(unittest.TestCase):
    def runtime(self):
        torch=NS(threads=8,interop=8)
        torch.set_num_threads=Mock(side_effect=lambda count:setattr(torch,'threads',count))
        torch.get_num_interop_threads=lambda:torch.interop
        torch.set_num_interop_threads=Mock(side_effect=lambda count:setattr(torch,'interop',count))
        cv2=NS(threads=8)
        cv2.setNumThreads=Mock(side_effect=lambda count:setattr(cv2,'threads',count))
        pools=NS(threads=8)
        def limit(*, limits):
            pools.threads=limits
            return object()
        limiter=Mock(side_effect=limit)
        with patch('nav2.semantic_runtime.mp.parent_process',return_value=object()), \
             patch.object(SemanticRuntime,'_lower_priority'),patch.dict(os.environ):
            runtime=SemanticRuntime(torch,cv2,limiter)
        return runtime,torch,cv2,pools,limiter

    def test_parent_process_is_rejected_before_any_global_changes(self):
        torch=Mock();cv2=Mock();limiter=Mock()
        with patch('nav2.semantic_runtime.mp.parent_process',return_value=None), \
             patch.object(SemanticRuntime,'_lower_priority') as lower,patch.dict(os.environ):
            before=dict(os.environ)
            with self.assertRaisesRegex(RuntimeError,'只能在自有子进程'):
                SemanticRuntime(torch,cv2,limiter)
            self.assertEqual(dict(os.environ),before)
        lower.assert_not_called();torch.set_num_threads.assert_not_called()
        cv2.setNumThreads.assert_not_called();limiter.assert_not_called()

    def test_first_prediction_and_backend_rebuild_are_single_threaded(self):
        runtime,torch,cv2,pools,limiter=self.runtime()
        counts=[];setup=[];validator=object()
        class Predictor:
            def setup_model(self, **kwargs):
                # Reproduce installed select_device('cpu') and native setup.
                torch.set_num_threads(7);cv2.threads=7;pools.threads=7
                setup.append(kwargs)
                return 'initialized'
            def infer(self):
                counts.append((torch.threads,torch.interop,cv2.threads,pools.threads))
                return 'result'
        class PromptFreeYOLOE:
            def __init__(self,path):self.path=path;self.predictor=None
            def _smart_load(self,key):return Predictor if key=='predictor' else validator
            def predict(self, image, predictor=None, rebuild=False):
                # Prompt-free YOLOE drops the explicit predictor parameter.
                if self.predictor is None or rebuild:
                    self.predictor=self._smart_load('predictor')()
                    self.predictor.setup_model(model='child model')
                return self.predictor.infer()
        model=runtime.model(PromptFreeYOLOE,'owned.pt')
        self.assertEqual(model.predict(object()),'result')
        self.assertEqual(model.predict(object(),rebuild=True),'result')
        self.assertEqual(counts,[(1,1,1,1),(1,1,1,1)])
        self.assertEqual(len(setup),2)
        self.assertIs(model._smart_load('validator'),validator)
        self.assertIs(model._smart_load('predictor'),model._smart_load('predictor'))
        self.assertTrue(issubclass(model._smart_load('predictor'),Predictor))
        torch.set_num_interop_threads.assert_called_once_with(1)
        self.assertEqual(limiter.call_count,3)

    def test_existing_single_interop_setting_is_not_set_again(self):
        torch=Mock();torch.get_num_interop_threads.return_value=1
        with patch('nav2.semantic_runtime.mp.parent_process',return_value=object()), \
             patch.object(SemanticRuntime,'_lower_priority'),patch.dict(os.environ):
            runtime=SemanticRuntime(torch,Mock(),Mock())
            runtime.apply();runtime.apply()
        torch.set_num_interop_threads.assert_not_called()

    def test_environment_limits_are_child_local_and_cover_future_imports(self):
        torch=Mock();torch.get_num_interop_threads.return_value=1
        with patch('nav2.semantic_runtime.mp.parent_process',return_value=object()), \
             patch.object(SemanticRuntime,'_lower_priority'),patch.dict(os.environ):
            SemanticRuntime(torch,Mock(),Mock())
            for name in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS',
                         'NUMEXPR_NUM_THREADS','NUMEXPR_MAX_THREADS','VECLIB_MAXIMUM_THREADS'):
                self.assertEqual(os.environ[name],'1')

    def test_priority_covers_existing_native_threads_and_never_raises_priority(self):
        with patch('nav2.semantic_runtime.os.listdir',return_value=['101','102','103']), \
             patch('nav2.semantic_runtime.os.getpriority',side_effect=[0,0,15,ProcessLookupError()]), \
             patch('nav2.semantic_runtime.os.setpriority') as setpriority:
            SemanticRuntime._lower_priority()
        self.assertEqual(setpriority.call_args_list,
                         [unittest.mock.call(os.PRIO_PROCESS,0,10),
                          unittest.mock.call(os.PRIO_PROCESS,101,10)])

    def test_existing_priority_below_normal_is_preserved(self):
        with patch('nav2.semantic_runtime.os.listdir',return_value=[]), \
             patch('nav2.semantic_runtime.os.getpriority',return_value=15), \
             patch('nav2.semantic_runtime.os.setpriority') as setpriority:
            SemanticRuntime._lower_priority()
        setpriority.assert_not_called()

    def test_source_expiring_during_preprocessing_never_reaches_forward(self):
        from nav2.semantic_request import RequestFreshness,SemanticRequestExpired
        runtime,_,_,_,_=self.runtime()
        freshness=RequestFreshness();runtime.before_inference=freshness
        forward=Mock(return_value='fresh result')
        class Predictor:
            def setup_model(self):return None
            def inference(self, image):return forward(image)
        predictor=runtime.predictor_type(Predictor)()
        predictor.setup_model()
        with patch('nav2.semantic_request.time.monotonic',return_value=100.):
            freshness.begin(100.)
            freshness(predictor)  # The on_predict_start check passes.
        with patch('nav2.semantic_request.time.monotonic',return_value=101.21):
            with self.assertRaises(SemanticRequestExpired):
                predictor.inference('expired image')
        forward.assert_not_called()
        with patch('nav2.semantic_request.time.monotonic',return_value=102.):
            freshness.begin(102.)
            self.assertEqual(predictor.inference('fresh image'),'fresh result')
        forward.assert_called_once_with('fresh image')
