# 【内容标注 / R38】精确分组计算最近障碍碎片，减少重复百分位扫描；不改变筛选阈值。
"""Exact fragment selection with one grouped sort instead of per-label scans."""
import numpy as np


def _unusual_points(labels, forward, lateral, ids):
    """Retain the original NumPy/NaN semantics outside the finite depth path."""
    distance = {i: float(np.percentile(forward[labels == i], 10)) for i in ids}
    seed = min(ids, key=distance.__getitem__)
    selected = labels == seed
    seed_xy = np.column_stack((forward[selected], lateral[selected]))
    lo, hi = seed_xy.min(axis=0), seed_xy.max(axis=0)
    for label in ids:
        if label == seed or abs(distance[label] - distance[seed]) > .30:
            continue
        candidate = labels == label
        xy = np.column_stack((forward[candidate], lateral[candidate]))
        gap = np.maximum(0, np.maximum(lo - xy.max(axis=0), xy.min(axis=0) - hi))
        if np.linalg.norm(gap) <= .20:
            selected |= candidate
    return selected


def nearest_fragments(labels, forward, lateral):
    """Keep the same nearest fragments, without filling unobserved pixels.

    Positive labels are ordered exactly as np.unique in the original selector.
    The tenth percentile retains NumPy's default linear interpolation, including
    its alternate subtraction order at weights >= 0.5. Group bounds always refer
    to the original seed; accepting a fragment never expands that seed's bounds.
    """
    positive = labels > 0
    if not positive.any():
        return positive
    group = labels[positive]
    x, y = forward[positive], lateral[positive]
    # Normal depth projection already excludes nonfinite points. Preserve the
    # old helper's behavior for standalone callers with unusual input as well.
    if not np.isfinite(x).all() or not np.isfinite(y).all():
        return _unusual_points(labels, forward, lateral, np.unique(group))

    order = np.lexsort((x, group))
    group, x, y = group[order], x[order], y[order]
    ids, starts, counts = np.unique(group, return_index=True, return_counts=True)

    index = (counts - 1) * .1
    below = np.floor(index).astype(np.intp)
    above = np.minimum(below + 1, counts - 1)
    weight = index - below
    a, b = x[starts + below], x[starts + above]
    difference = np.subtract(b, a)
    distance = np.add(a, difference * weight)
    np.subtract(b, difference * (1 - weight), out=distance, where=weight >= .5)
    # The original stores float(np.percentile(...)), including long-double input.
    distance = distance.astype(float, copy=False)
    if not np.isfinite(distance).all():
        # Extreme finite standalone input can overflow interpolation. Python's
        # original min() ordering for NaN differs from np.argmin().
        return _unusual_points(labels, forward, lateral, ids)
    seed = int(np.argmin(distance))

    lo = np.column_stack((x[starts], np.minimum.reduceat(y, starts)))
    hi = np.column_stack((x[starts + counts - 1], np.maximum.reduceat(y, starts)))
    eligible = np.flatnonzero(np.abs(distance - distance[seed]) <= .30)
    chosen = np.zeros(len(ids), dtype=bool)
    chosen[seed] = True
    # Retain the exact scalar norm operation at the 0.20 m boundary. Computing
    # one tiny norm per eligible label is cheap; repeated percentile setup and
    # repeated full-image label scans were the expensive work.
    for candidate in eligible:
        if candidate == seed:
            continue
        gap = np.maximum(0, np.maximum(lo[seed] - hi[candidate],
                                      lo[candidate] - hi[seed]))
        chosen[candidate] = np.linalg.norm(gap) <= .20
    return np.isin(labels, ids[chosen])
