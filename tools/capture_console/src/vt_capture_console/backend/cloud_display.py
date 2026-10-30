"""Camera-local preview cropping; source frames and their paired poses stay intact."""
from __future__ import annotations


class WristCloudDisplay:
    RANGE_M = (.07, .50)

    def __init__(self):
        self._cache = {}

    def clear(self):
        self._cache.clear()

    def select(self, clouds, cameras, enabled):
        wrists = {c['id'] for c in cameras
                  if c.get('tracker_role') in ('left_wrist', 'right_wrist')} if enabled else set()
        self._cache = {name: value for name, value in self._cache.items()
                       if name in clouds and name in wrists}
        result = dict(clouds)
        for name in wrists & clouds.keys():
            cloud = clouds[name]
            positions, colors = cloud['positions'], cloud['colors']
            cached = self._cache.get(name)
            if cached is None or cached[0] is not positions or cached[1] is not colors:
                # Cloud coordinates are still local to this camera, before
                # world_from_color and the display/world rotation are applied.
                low, high = self.RANGE_M
                keep = (positions[:, 2] >= low) & (positions[:, 2] <= high)
                cached = (positions, colors, positions[keep], colors[keep])
                self._cache[name] = cached
            result[name] = dict(cloud, positions=cached[2], colors=cached[3],
                                display_range_m=self.RANGE_M)
        return result
