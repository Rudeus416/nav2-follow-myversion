import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from nav2_map_edit import MapEdits


def base():
    return NS(header=NS(frame_id='map'), info=NS(width=4, height=3, resolution=0.05,
        origin=NS(position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.))),
        data=[100, 0, 0, 100, 100, 0, 0, 100, 100, 0, 0, 100])


class MapEditsTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.editor = MapEdits(self.tmp.name)
        self.original = base()
        self.editor.set_base(self.original)

    def edit(self, action, rect=None):
        state = self.editor.state()
        return self.editor.edit(action, rect, state['base_id'], state['revision'])

    def test_freehand_persists_and_undoes_as_one_action(self):
        self.edit('add', [1, 0, 2, 1])
        before = self.editor.compose().data
        self.edit('add', {'cells': [[2, 0], [2, 1], [1, 2], [2, 1]]})
        self.assertEqual(len(self.editor.state()['rectangles'][-1]['cells']), 3)
        loaded = MapEdits(self.tmp.name)
        loaded.set_base(self.original)
        self.assertEqual(loaded.compose().data, self.editor.compose().data)
        self.assertEqual(loaded.compose().data[6], 100)
        self.edit('undo')
        self.assertEqual(self.editor.compose().data, before)
        for cells in ([], [[-1, 0]], [[4, 0]], [[1.1, 0]], [[True, 0]]):
            with self.assertRaises(ValueError):
                self.edit('add', {'cells': cells})

    def test_add_reload_undo_and_clear_preserve_walls(self):
        original = list(self.original.data)
        self.edit('add', [1, 0, 2, 2])
        self.assertEqual(self.editor.compose().data[1], 100)
        self.assertEqual(self.original.data, original)
        loaded = MapEdits(self.tmp.name)
        loaded.set_base(self.original)
        self.assertEqual(loaded.compose().data, self.editor.compose().data)
        self.edit('add', [2, 0, 3, 1])
        self.edit('undo')
        self.assertEqual(self.editor.compose().data[2], 0)
        self.assertEqual(self.editor.compose().data[1], 100)
        self.edit('clear')
        self.assertEqual(self.editor.compose().data, original)

    def test_reject_invalid_rect_and_stale_revision(self):
        old = self.editor.state()
        for rect in ([0, 0, 5, 1], [1, 1, 1, 2], [0, -1, 1, 1], [True, 0, 2, 2]):
            with self.assertRaises(ValueError):
                self.edit('add', rect)
        self.edit('add', [1, 0, 2, 1])
        with self.assertRaises(ValueError):
            self.editor.edit('clear', None, old['base_id'], old['revision'])

    def test_different_map_does_not_reuse_rectangles(self):
        self.edit('add', [1, 0, 2, 1])
        other = base()
        other.info.origin.position.x = 1.0
        self.editor.set_base(other)
        self.assertEqual(self.editor.state()['rectangles'], [])

    def test_corrupt_saved_file_blocks_publication(self):
        path = Path(self.tmp.name) / (self.editor.base_id + '.json')
        path.write_text('invalid')
        loaded = MapEdits(self.tmp.name)
        with self.assertRaises(ValueError):
            loaded.set_base(self.original)
        self.assertFalse(loaded.state()['ready'])
        with self.assertRaises(ValueError):
            loaded.compose()

class MapEditApiTests(unittest.TestCase):
    def test_backend_rejects_moving_and_radar_only_edits(self):
        import threading
        import control
        from fastapi import FastAPI, HTTPException
        from unittest.mock import Mock, patch
        motion = NS(lock=threading.RLock(), estop=False,
                    navigation=NS(obstacle_mode='map', clear_preview=Mock()))
        view = Mock()
        with patch.object(control, 'make_app', return_value=FastAPI()), patch.object(control, 'ConfigurationStore'):
            app = control.build_app(None, None, None, None, None, motion, view)
        endpoint = next(r.endpoint for r in app.routes if r.path == '/api/nav2/map-edit')
        payload = dict(action='add', rect=[1, 1, 2, 2], base_id='test', revision=1)
        with self.assertRaises(HTTPException) as caught:
            endpoint(payload)
        self.assertEqual(caught.exception.status_code, 409)
        view.edit_map.assert_not_called()
        motion.estop = True
        motion.navigation.obstacle_mode = 'radar'
        with self.assertRaises(HTTPException):
            endpoint(payload)
        view.edit_map.assert_not_called()
        motion.navigation.obstacle_mode = 'both'
        endpoint(payload)
        view.edit_map.assert_called_once_with('add', [1, 1, 2, 2], 'test', 1)
        motion.navigation.clear_preview.assert_called_once()


class MapPublisherTests(unittest.TestCase):
    def test_publisher_reloads_saved_blocks_and_removals(self):
        from unittest.mock import Mock, patch
        from nav2_map_publisher import EditedMapPublisher
        with tempfile.TemporaryDirectory() as directory:
            message = base()
            editor = MapEdits(directory)
            editor.set_base(message)
            state = editor.state()
            editor.edit('add', [1, 0, 2, 1], state['base_id'], state['revision'])
            node = NS(base=message, publisher=Mock(),
                      get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: NS(sec=1, nanosec=0))),
                      get_logger=lambda: Mock())
            with patch('nav2_map_publisher.MapEdits', side_effect=lambda _: MapEdits(directory)):
                EditedMapPublisher.reload(node)
                self.assertEqual(node.publisher.publish.call_args.args[0].data[1], 100)
                state = editor.state()
                editor.edit('clear', None, state['base_id'], state['revision'])
                EditedMapPublisher.reload(node)
                self.assertEqual(node.publisher.publish.call_args.args[0].data, message.data)


if __name__ == '__main__':
    unittest.main()
