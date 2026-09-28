# 【内容标注】用途：地图模式全局规划窗口覆盖整张真实静态地图。
# 对应用户需求：R01 R13 R20 R22 R26（见 nav2/CODE_GUIDE.md）。
# 添加逻辑：按实际地图的旋转后跨度扩大全局滚动窗；不改局部窗、起点或地图占用。
"""Size the global rolling window from map geometry, without inventing free space."""
import math
from pathlib import Path

import yaml


MAX_MAP_PIXELS = 1_000_000
MAX_GLOBAL_CELLS = 1_000_000


def _number(value, label, positive=False):
    if (type(value) not in (int, float) or not math.isfinite(value)
            or (positive and value <= 0)):
        raise ValueError(f'{label}必须是有限' + ('正数' if positive else '数值'))
    return float(value)


def configure_global_map_window(config, mode, map_file, map_to_odom_yaw=math.pi / 2):
    """Expand map/both global windows, retaining larger explicitly set dimensions.

    For any robot center inside the map, the maximum axis separation from a map
    point is the full rotated map span, not half of it. A centered rolling window
    therefore needs twice that span. Extra map/costmap cells cover the separate
    boundary layer and costmap origin rounding. Map pixels, transforms, static
    layers, unknown-space policy and the local controller window stay unchanged.
    """
    if mode == 'radar':
        return None
    if mode not in ('map', 'both'):
        raise ValueError('NAV2_OBSTACLE_MODE must be radar, map or both')
    path = Path(map_file)
    try:
        metadata = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f'无法读取地图配置 {path}: {exc}') from exc
    if not isinstance(metadata, dict):
        raise ValueError('地图配置必须是 YAML 字典')
    resolution = _number(metadata.get('resolution'), '地图 resolution', positive=True)
    origin = metadata.get('origin')
    if not isinstance(origin, (list, tuple)) or len(origin) != 3:
        raise ValueError('地图 origin 必须为 [x, y, yaw]')
    for value in origin:
        _number(value, '地图 origin')
    image_file = metadata.get('image')
    if not isinstance(image_file, str) or not image_file.strip():
        raise ValueError('地图 image 必须指定图片文件')
    image_path = path.parent / image_file
    # Pillow is already available in the system launch interpreter. Reading
    # the header avoids allocating a full image just to obtain dimensions.
    try:
        from PIL import Image
    except ImportError as exc:
        raise ValueError('地图尺寸检查需要系统 Python 已安装的 Pillow') from exc
    try:
        with Image.open(image_path) as image:
            pixels_x, pixels_y = image.size
    except (OSError, ValueError, Image.DecompressionBombError) as exc:
        raise ValueError(f'无法读取地图图片尺寸 {image_path}: {exc}') from exc
    if not (0 < pixels_x * pixels_y <= MAX_MAP_PIXELS):
        raise ValueError(f'地图图片最多支持 {MAX_MAP_PIXELS} 个像素')
    costmap = config['global_costmap']['global_costmap']['ros__parameters']
    if costmap.get('global_frame') != 'odom' or costmap.get('rolling_window') is not True:
        raise ValueError('地图范围配置要求全局代价图使用 odom 和 rolling_window=true')
    cell = _number(costmap.get('resolution'), '全局代价图 resolution', positive=True)
    current = []
    for axis in ('width', 'height'):
        value = costmap.get(axis)
        if type(value) is not int or value <= 0:
            raise ValueError(f'全局代价图 {axis} 必须是正整数米')
        current.append(value)
    angle = origin[2] - _number(map_to_odom_yaw, 'map → odom yaw')
    c, s = abs(math.cos(angle)), abs(math.sin(angle))
    image_x, image_y = pixels_x * resolution, pixels_y * resolution
    span_x, span_y = c * image_x + s * image_y, s * image_x + c * image_y
    margin = 2 * resolution + 2 * cell
    if not all(math.isfinite(v) for v in (span_x, span_y, margin, 2 * (span_x + margin), 2 * (span_y + margin))):
        raise ValueError('地图比例或分辨率过大，无法计算有限的规划窗口')
    sizes = [max(previous, math.ceil(2 * (span + margin)))
             for previous, span in zip(current, (span_x, span_y))]
    if (any(size > MAX_GLOBAL_CELLS * cell for size in sizes)
            or math.ceil(sizes[0] / cell) * math.ceil(sizes[1] / cell) > MAX_GLOBAL_CELLS):
        raise ValueError(
            f'覆盖整张地图需要全局窗口 {sizes[0]}×{sizes[1]} 米，'
            f'按分辨率 {cell:g} 米超过 {MAX_GLOBAL_CELLS} 格上限；'
            '请分区使用真实地图或检查地图比例，不能把图外当作空地')
    costmap['width'], costmap['height'] = sizes
    # A rolling costmap update does not carry its origin. Publish complete global
    # snapshots for whole-route checks, so no increment can use an old window.
    costmap['always_send_full_costmap'] = True
    return {'width': sizes[0], 'height': sizes[1],
            'span_x': span_x, 'span_y': span_y, 'margin': margin}
