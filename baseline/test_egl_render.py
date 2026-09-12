#!/usr/bin/env python3
"""验证 headless 离屏渲染后端（EGL / osmesa）在容器内是否可用。

用途：WSLg 的 GL 上下文不可靠（GUI 合成改动后 mjr_render 崩溃），
改走 headless + 离屏渲染 + 相机图像推流，需要确认后端能跑。
"""
import os
import sys

print("MUJOCO_GL =", os.environ.get("MUJOCO_GL", "(unset)"), flush=True)

import mujoco  # noqa: E402

xml = """
<mujoco>
  <worldbody>
    <light pos="0 0 1"/>
    <geom type="box" size=".1 .1 .1" rgba="1 0 0 1"/>
    <camera name="cam0" pos="0.6 0.6 0.6" xyaxes="1 -1 0 0 0 1"/>
  </worldbody>
</mujoco>
"""
try:
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, 64, 64)
    renderer.update_scene(data)
    img = renderer.render()
    print("离屏渲染 OK:", img.shape, "均值", float(img.mean()).__round__(1), flush=True)
    # 再测指定相机
    renderer.update_scene(data, camera="cam0")
    img2 = renderer.render()
    print("指定相机渲染 OK:", img2.shape, flush=True)
except Exception as exc:  # noqa: BLE001
    print("离屏渲染失败:", type(exc).__name__, str(exc)[:300], flush=True)
    sys.exit(1)
