#!/usr/bin/env python3
"""可配置 YOLO 感知适配器。

复用官方 kele_detect.py 的 ROS/RGB-D/坐标变换，只替换 checkpoint 和阈值。
最终运行时不读取 GT，不访问布局文件。
"""
from __future__ import annotations
import os, runpy, sys
PERCEPTION_DIR='/workspace/supermarket_sorting_task/examples/supermarket_sorting/perception'
sys.path.insert(0, PERCEPTION_DIR)
import backends

Original = backends.YoloBackend
class ConfigurableYoloBackend(Original):
    def __init__(self, ckpt_path: str, conf_thresh: float = 0.65, device: str = 'auto'):
        ckpt=os.getenv('KELE_CHECKPOINT',ckpt_path)
        conf=float(os.getenv('KELE_CONF',str(conf_thresh)))
        print(f'[finetuned_yolo] checkpoint={ckpt} conf={conf:.3f}',flush=True)
        super().__init__(ckpt,conf_thresh=conf,device=device)
backends.YoloBackend=ConfigurableYoloBackend
runpy.run_path(os.path.join(PERCEPTION_DIR,'kele_detect.py'),run_name='__main__')
