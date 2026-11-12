# 主动感知 SDK

上层用底部广角相机的水平角度线索选择下一个观察方向。局部相机确认目标坐标后，上层调用拾取规划 SDK。主动感知 SDK 使用 Python 3.10 或以上版本，运行时只使用标准库，可独立安装。

```bash
python -m pip install /path/to/active_perception_sdk-0.1.0-py3-none-any.whl
```

也可以解压源码包，在解压后的目录运行 `python -m pip install .`。

```python
from math import degrees, radians
from active_perception_sdk import ObservationCue, suggest_views

suggestions = suggest_views(
    [ObservationCue(radians(38), (1.2, 0.9)), ObservationCue(radians(55)), ObservationCue(radians(-75))],
    horizontal_fov=radians(30),
)
print([round(degrees(view.yaw_offset), 1) for view in suggestions])  # [46.5, -75.0]
```

- `ObservationCue.bearing` 是相对于当前局部相机光轴的水平角度线索，左为正，单位是弧度。可选的 `approximate_xy` 是以同一观察位置为原点的粗坐标，单位是米。上层负责把底部相机的结果换算到这个参考系。
- `horizontal_fov` 是局部相机有效水平视场的**全角**，单位是弧度。示例中的 30° 是演示值。
- 候选按 `yaw_offset` 的绝对值从小到大排序。转角单位是弧度，范围为 `[-π, π)`，`0` 表示原地重新观测。`cue_indices` 对应本次输入线索的下标，`estimated_nearest_distance_m` 来自这组线索中最近的粗坐标，仅用于估计距离。上层选择可安全执行的候选，完成后刷新观测。
- 同一观察位置内，上层可以用 `tried_yaw_offsets` 传入本轮已尝试过的朝向，SDK 会跳过这些视场已覆盖的角度。每次调用都以**当前局部相机光轴**为零度，上层须换算历史朝向。平移后使用新观测，清空旧的尝试记录。
- 空元组表示这一轮没有剩余角度线索。遮挡可能让两台相机都漏检，上层可在换一个观察位置后再次调用。

`examples/quick_start.py` 用二维几何模拟视场、探测距离、圆形障碍物遮挡和远处坐标偏差。它演示原地换向找到近目标，以及在第二个观测位置看到原来被挡住的目标。

从仓库根目录构建和测试：

```bash
uv build --sdist --wheel --out-dir sim/output/active-perception-sdk active_perception
PYTHONPATH=active_perception/src python3.10 -m unittest discover -s active_perception/tests -v
```
