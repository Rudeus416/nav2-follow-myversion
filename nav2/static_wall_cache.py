# 【内容标注 / R35】缓存未变化的静态墙和外侧边界，减少每帧视觉处理占用。
# 对应用户：行驶途中视觉超时停车，“怎么修改”；仅优化自有导航计算。
"""Exact static raster reuse; live visual obstacles are never cached here."""
import numpy as np


# 【职责 / R35】StaticWallCache：缓存相同静态墙的栅格化结果，视觉点仍逐帧叠加。
class StaticWallCache:
    # 【职责 / R35】__init__：初始化空键和值；不预取地图或续期证据。
    def __init__(self):
        self._key=None
        self._data=None

    # 【职责 / R35】raster：地图内容、几何、TF 或边界变化才重建，并向调用方返回可写副本。
    def raster(self, shape, origin, resolution, grid, transform, boundary=False):
        """Return a writable copy, invalidating on content/geometry/TF changes.

        Ignore receipt stamps: repeated identical map messages are not edits.
        Keep exact geometry (no rounding), including rotated maps and map->odom.
        Compare occupied cells, matching merge_static_walls' >=65 rule exactly.
        """
        from nav2.vision_layer import merge_static_walls
        pose=grid.info.origin
        q,t=transform.rotation,transform.translation
        p,o=pose.position,pose.orientation
        occupied=(np.asarray(grid.data)>=65).tobytes()
        key=(tuple(shape),tuple(origin),resolution,boundary,
             grid.info.width,grid.info.height,grid.info.resolution,
             p.x,p.y,o.x,o.y,o.z,o.w,t.x,t.y,q.x,q.y,q.z,q.w,occupied)
        if key!=self._key:
            data=np.zeros(shape,np.int8)
            merge_static_walls(data,origin,resolution,grid,transform)
            if boundary:
                from nav2.boundary_map import boundary_grid
                merge_static_walls(data,origin,resolution,boundary_grid(grid),transform)
            # Publish cache only after all work succeeds; errors never serve old walls.
            self._key,self._data=key,data
        # Each frame adds only its own visual cells to this copy. Old obstacles
        # must not leave trails or persist after a clear/empty observation.
        return self._data.copy()
