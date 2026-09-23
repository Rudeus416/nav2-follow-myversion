"""Compose user rectangles with an immutable occupancy map; persist separately."""
import copy
import hashlib
import json
import os
import threading
from pathlib import Path


class MapEdits:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.lock = threading.RLock()
        self.base = None
        self.base_id = ''
        self.rectangles = []
        self.revision = 0

    def validate(self, rect):
        if isinstance(rect, dict):
            cells = rect.get('cells')
            if not isinstance(cells, list) or not 1 <= len(cells) <= 20000:
                raise ValueError('一笔绘画需包含 1 至 20000 个栅格')
            for cell in cells:
                if (not isinstance(cell, list) or len(cell) != 2
                        or any(type(v) is not int for v in cell)
                        or not (0 <= cell[0] < self.base.info.width and 0 <= cell[1] < self.base.info.height)):
                    raise ValueError('画笔栅格坐标无效或超出地图')
            return {'cells': [list(c) for c in sorted(set(map(tuple, cells)))]}
        if not isinstance(rect, list) or len(rect) != 4 or any(type(v) is not int for v in rect):
            raise ValueError('阻挡区必须是四个整数栅格坐标')
        x0, y0, x1, y1 = rect
        if not (0 <= x0 < x1 <= self.base.info.width and 0 <= y0 < y1 <= self.base.info.height):
            raise ValueError('阻挡区超出地图或面积为零')
        return rect

    def set_base(self, msg):
        with self.lock:
            info = msg.info
            if not (0 < info.width * info.height <= 1000000 and info.resolution > 0
                    and len(msg.data) == info.width * info.height):
                raise ValueError('底图尺寸无效')
            p, q = info.origin.position, info.origin.orientation
            geometry = [msg.header.frame_id, info.width, info.height, info.resolution,
                        p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            key = hashlib.sha256(json.dumps(geometry).encode() + bytes(v + 1 for v in msg.data)).hexdigest()
            if key == self.base_id:
                return
            self.base = copy.deepcopy(msg)
            self.base_id = ''
            self.rectangles = []
            path = self.directory / (key + '.json')
            if path.exists():
                saved = json.loads(path.read_text())
                if not isinstance(saved, list) or len(saved) > 200:
                    raise ValueError('保存的阻挡区文件无效')
                self.rectangles = [self.validate(rect) for rect in saved]
            self.base_id = key
            self.revision += 1

    def state(self):
        with self.lock:
            return dict(ready=bool(self.base_id), base_id=self.base_id,
                        revision=self.revision, rectangles=copy.deepcopy(self.rectangles))

    def compose(self):
        with self.lock:
            if not self.base_id:
                raise ValueError('静态地图尚未就绪')
            msg = copy.deepcopy(self.base)
            data = list(msg.data)
            for shape in self.rectangles:
                if isinstance(shape, dict):
                    for x, y in shape['cells']:
                        data[y * msg.info.width + x] = 100
                else:
                    x0, y0, x1, y1 = shape
                    for y in range(y0, y1):
                        data[y * msg.info.width + x0:y * msg.info.width + x1] = [100] * (x1-x0)
            msg.data = data
            return msg

    def edit(self, action, rect, base_id, revision):
        with self.lock:
            if not self.base_id or base_id != self.base_id or revision != self.revision:
                raise ValueError('地图已变化，请刷新地图后重新编辑')
            updated = copy.deepcopy(self.rectangles)
            if action == 'add':
                if len(updated) >= 200:
                    raise ValueError('最多添加 200 个阻挡区')
                updated.append(self.validate(rect))
            elif action == 'undo':
                if updated:
                    updated.pop()
            elif action == 'clear':
                updated = []
            else:
                raise ValueError('未知地图编辑操作')
            self.directory.mkdir(parents=True, exist_ok=True)
            path = self.directory / (self.base_id + '.json')
            tmp = path.with_suffix('.tmp')
            tmp.write_text(json.dumps(updated) + '\n')
            os.replace(tmp, path)
            self.rectangles = updated
            self.revision += 1
            return self.compose()
