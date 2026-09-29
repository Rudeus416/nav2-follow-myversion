"""Generation safety without waiting for the processing/fusion engine lock."""
import threading
import unittest
from dataclasses import FrozenInstanceError
from types import SimpleNamespace
from unittest.mock import Mock

from nav2.vision_epoch import VisionEpoch


class Engine:
    def __init__(self):
        self.lock = threading.RLock()
        self._config_version = 1
        self._stream_epoch = 'camera-a'
        self.error = None

    def configure(self, value, *, epoch=None):
        with self.lock:
            self._config_version += 1
            if epoch is not None:
                self._stream_epoch = epoch
            if self.error is not None:
                raise self.error
            return value


class VisionEpochTests(unittest.TestCase):
    def test_initial_generation_is_immutable_and_supports_static_fixture(self):
        engine = SimpleNamespace(lock=threading.RLock(), _config_version=3,
                                 _stream_epoch='read-only')
        gate = VisionEpoch(engine)
        self.addCleanup(gate.close)
        snapshot = gate.snapshot()
        self.assertEqual((snapshot.version, snapshot.epoch, snapshot.ready),
                         (3, 'read-only', True))
        self.assertTrue(gate.current(snapshot))
        self.assertFalse(gate.current(None))
        with self.assertRaises(FrozenInstanceError):
            snapshot.version = 4

    def test_configuration_invalidates_before_waiting_for_engine(self):
        engine = Engine()
        invalidated = threading.Event()
        gate = VisionEpoch(engine, invalidated.set)
        self.addCleanup(gate.close)
        old = gate.snapshot()
        errors = []
        finished = threading.Event()

        def configure():
            try:
                self.assertEqual(engine.configure('value', epoch='camera-b'), 'value')
            except BaseException as error:
                errors.append(error)
            finally:
                finished.set()

        with engine.lock:
            worker = threading.Thread(target=configure, daemon=True)
            worker.start()
            self.assertTrue(invalidated.wait(1.))
            read_done = threading.Event()
            readings = []
            def read():
                readings.append((gate.snapshot(), gate.current(old)))
                read_done.set()
            reader = threading.Thread(target=read, daemon=True)
            reader.start()
            self.assertTrue(read_done.wait(.5))
            self.assertEqual(readings, [(None, False)])
            self.assertFalse(finished.is_set())
        reader.join(1.)
        self.assertTrue(finished.wait(1.))
        worker.join(1.)
        self.assertEqual(errors, [])
        new = gate.snapshot()
        self.assertEqual((new.version, new.epoch), (2, 'camera-b'))
        self.assertGreater(new.token, old.token)

    def test_ready_reads_do_not_acquire_engine_lock(self):
        engine = Engine()
        gate = VisionEpoch(engine)
        self.addCleanup(gate.close)
        held, release = threading.Event(), threading.Event()

        def holder():
            with engine.lock:
                held.set()
                release.wait(2.)

        worker = threading.Thread(target=holder, daemon=True)
        worker.start()
        try:
            self.assertTrue(held.wait(1.))
            read_done = threading.Event()
            readings = []
            def read():
                readings.append(gate.current(gate.snapshot()))
                read_done.set()
            reader = threading.Thread(target=read, daemon=True)
            reader.start()
            self.assertTrue(read_done.wait(.5))
            self.assertEqual(readings, [True])
        finally:
            release.set()
            worker.join(1.)
            reader.join(1.)

    def test_original_exception_propagates_and_remains_closed_until_success(self):
        engine = Engine()
        gate = VisionEpoch(engine)
        self.addCleanup(gate.close)
        original = gate.snapshot()
        error = RuntimeError('original configuration failed')
        engine.error = error
        with self.assertRaises(RuntimeError) as caught:
            engine.configure('failed', epoch='camera-b')
        self.assertIs(caught.exception, error)
        self.assertIsNone(gate.snapshot())
        self.assertFalse(gate.current(original))
        engine.error = None
        self.assertEqual(engine.configure('recovered'), 'recovered')
        self.assertEqual(gate.snapshot().version, 3)

    def test_callback_runs_without_gate_or_engine_lock(self):
        engine = Engine()
        observations = []

        def invalidate():
            observations.append(gate.snapshot())
            acquired = []
            def read_engine():
                with engine.lock:
                    acquired.append(True)
            reader = threading.Thread(target=read_engine, daemon=True)
            reader.start()
            reader.join(1.)
            self.assertEqual(acquired, [True])

        gate = VisionEpoch(engine, invalidate)
        self.addCleanup(gate.close)
        engine.configure('next')
        self.assertEqual(observations, [None])

    def test_concurrent_configurations_with_existing_engine_lock_do_not_deadlock(self):
        engine = Engine()
        first_entered, both_entered = threading.Event(), threading.Event()
        count_lock = threading.Lock()
        count = 0

        def invalidate():
            nonlocal count
            with count_lock:
                count += 1
                (first_entered if count == 1 else both_entered).set()

        gate = VisionEpoch(engine, invalidate)
        self.addCleanup(gate.close)
        errors = []
        completed = threading.Event()

        def first():
            try:
                engine.configure('first', epoch='camera-final')
            except BaseException as error:
                errors.append(error)

        def outer_lock_holder():
            try:
                with engine.lock:
                    worker = threading.Thread(target=first, daemon=True)
                    worker.start()
                    self.assertTrue(first_entered.wait(1.))
                    engine.configure('second', epoch='camera-intermediate')
                    self.assertTrue(both_entered.is_set())
                    self.assertIsNone(gate.snapshot())
                worker.join(1.)
                self.assertFalse(worker.is_alive())
            except BaseException as error:
                errors.append(error)
            finally:
                completed.set()

        holder = threading.Thread(target=outer_lock_holder, daemon=True)
        holder.start()
        self.assertTrue(completed.wait(2.))
        holder.join(1.)
        self.assertEqual(errors, [])
        self.assertEqual((gate.snapshot().version, gate.snapshot().epoch),
                         (3, 'camera-final'))

    def test_failed_overlapping_configuration_keeps_entire_batch_closed(self):
        engine = Engine()
        entered, release = threading.Event(), threading.Event()
        original = engine.configure
        expected = RuntimeError('overlapping failure')

        def configure(value):
            if value == 'failed':
                entered.set()
                release.wait(2.)
                raise expected
            return original(value)

        engine.configure = configure
        gate = VisionEpoch(engine)
        self.addCleanup(gate.close)
        errors = []

        def fail():
            try:
                engine.configure('failed')
            except BaseException as error:
                errors.append(error)

        worker = threading.Thread(target=fail, daemon=True)
        worker.start()
        try:
            self.assertTrue(entered.wait(1.))
            engine.configure('succeeded')
            self.assertIsNone(gate.snapshot())
        finally:
            release.set()
            worker.join(1.)
        self.assertEqual(errors, [expected])
        self.assertIsNone(gate.snapshot())
        engine.configure('recover')
        self.assertIsNotNone(gate.snapshot())

    def test_invalidation_callback_failure_is_closed_and_does_not_call_original(self):
        engine = Engine()
        error = RuntimeError('cannot invalidate navigation')
        callback = Mock(side_effect=error)
        gate = VisionEpoch(engine, callback)
        self.addCleanup(gate.close)
        with self.assertRaises(RuntimeError) as caught:
            engine.configure('unused')
        self.assertIs(caught.exception, error)
        self.assertEqual(engine._config_version, 1)
        self.assertIsNone(gate.snapshot())

    def test_close_during_configuration_prevents_late_republication(self):
        engine = Engine()
        original = engine.configure
        entered, release = threading.Event(), threading.Event()
        def invalidate():
            entered.set()
            release.wait(2.)
        gate = VisionEpoch(engine, invalidate)
        old = gate.snapshot()
        worker = threading.Thread(target=engine.configure, args=('late',), daemon=True)
        worker.start()
        try:
            self.assertTrue(entered.wait(1.))
            gate.close()
            self.assertEqual(engine.configure, original)
        finally:
            release.set()
            worker.join(1.)
        self.assertFalse(worker.is_alive())
        self.assertIsNone(gate.snapshot())
        self.assertFalse(gate.current(old))

    def test_close_restores_only_owned_wrapper_and_late_calls_keep_original(self):
        engine = Engine()
        original = engine.configure
        invalidated = Mock()
        gate = VisionEpoch(engine, invalidated)
        late = engine.configure
        gate.close()
        self.assertEqual(engine.configure, original)
        self.assertEqual(late('late'), 'late')
        invalidated.assert_not_called()
        self.assertIsNone(gate.snapshot())

        gate = VisionEpoch(engine)
        replacement = Mock()
        engine.configure = replacement
        gate.close()
        self.assertIs(engine.configure, replacement)


if __name__ == '__main__':
    unittest.main()
